#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include <locale.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <dirent.h>
#include <png.h>

#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/threadmgr.h>
#include <psp2/kernel/clib.h>
#include <psp2/ctrl.h>
#include <psp2/touch.h>
#include <psp2/motion.h>
#include <psp2/audioout.h>
#include <psp2/power.h>
#include <psp2/io/fcntl.h>
#include <psp2/io/dirent.h>
#include <psp2/io/stat.h>
#include <vitaGL.h>

// Force 32-bit ARM softfp calling convention for functions crossing the N900 ABI boundary
#define SOFTFP __attribute__((pcs("aapcs")))

// Memory configuration for Vita newlib + vitaGL
int _newlib_heap_size_user = 64 * 1024 * 1024;
unsigned int sceUserMainThreadStackSize = 2 * 1024 * 1024;

// ---------------------------------------------------------------------------
// Logging: one persistent file handle, timestamped, flushed per line.
// Chatty per-call logs (every fopen/fread/shader source) go through vlog() and
// only appear when the file ux0:data/BounceEvolution/debug.txt exists.
// ---------------------------------------------------------------------------
static FILE *g_logf = NULL;
static volatile int g_log_lock = 0;
static int g_verbose = 0;
static void vita_log(const char *fmt, ...) {
    while (__sync_lock_test_and_set(&g_log_lock, 1)) sceKernelDelayThread(50);
    if (!g_logf) g_logf = fopen("ux0:data/BounceEvolution/vita_log.txt", "a");
    if (g_logf) {
        fprintf(g_logf, "[%8u ms] ", (unsigned)(sceKernelGetProcessTimeLow() / 1000));
        va_list ap; va_start(ap, fmt); vfprintf(g_logf, fmt, ap); va_end(ap);
        fputc('\n', g_logf);
        fflush(g_logf);
    }
    __sync_lock_release(&g_log_lock);
}
#define vlog(...) do { if (g_verbose) vita_log(__VA_ARGS__); } while (0)

extern uint8_t bounce_rx_base[];
extern uint8_t bounce_rw_base[];
extern uint8_t bounce_x11_state[];
extern int bounce_main(int argc, char **argv);

// ============================================================================
// 1. Path Redirection (/opt/bounce/data -> ux0:data/BounceEvolution, /home/user -> ux0:data/BounceEvolution)
// ============================================================================
static void translate_path(const char *in, char *out, size_t out_sz) {
    if (!in) { out[0] = '\0'; return; }
    if (strncmp(in, "/var/opt/bounce/", 16) == 0) {
        snprintf(out, out_sz, "ux0:data/BounceEvolution/%s", in + 16);
    } else if (strncmp(in, "/opt/bounce/data/", 17) == 0) {
        snprintf(out, out_sz, "ux0:data/BounceEvolution/%s", in + 17);
    } else if (strncmp(in, "/opt/bounce/", 12) == 0) {
        snprintf(out, out_sz, "ux0:data/BounceEvolution/%s", in + 12);
    } else if (strncmp(in, "/home/user/", 11) == 0) {
        snprintf(out, out_sz, "ux0:data/BounceEvolution/%s", in + 11);
    } else if (strcmp(in, "/home/user") == 0) {
        snprintf(out, out_sz, "ux0:data/BounceEvolution");
    } else if (strncmp(in, "data/", 5) == 0) {
        snprintf(out, out_sz, "ux0:data/BounceEvolution/%s", in + 5);
    } else if (strncmp(in, "./data/", 7) == 0) {
        snprintf(out, out_sz, "ux0:data/BounceEvolution/%s", in + 7);
    } else if (in[0] != '/' && strstr(in, "ux0:") == NULL) {
        snprintf(out, out_sz, "ux0:data/BounceEvolution/%s", in);
    } else {
        snprintf(out, out_sz, "%s", in);
    }
}

// ============================================================================
// 2. Accelerometer Hook (Vita Motion + Left Analog Stick -> math::float3)
// ============================================================================

// ---------------------------------------------------------------------------
// The original game runs a thread that reads
//   /sys/class/i2c-adapter/i2c-3/3-001d/coord   ("x y z" in milli-g)
// every 33 ms and converts it to (-x, +y, -z)/1000 (raw) plus a low-pass
// filtered copy (f = 0.8 f + 0.2 raw).  We skip that thread (Accelerometer::start
// is patched to "return 1") and hook getData()/getDataFiltered() instead.
//
// Model taken from the Maemulator: a resting N900 reads (0, 0, -1000) mG; the
// player's input is turned into two rotation angles (about Y and about X) that
// rotate that vector; a jump is a short spike of -30 g on Z.  Here the angles
// come from the Vita's real tilt (relative to a calibrated neutral pose) plus
// the left stick / D-Pad, and Cross produces the jump spike.
//
//   SELECT          : recalibrate neutral pose (hold the Vita how you like)
//   SELECT + L      : flip left/right tilt direction   (saved to vita.cfg)
//   SELECT + R      : flip forward/back tilt direction (saved to vita.cfg)
// ---------------------------------------------------------------------------
static float g_roll_sign = 1.0f, g_pitch_sign = -1.0f, g_tilt_gain = 1.0f;
static int   g_audio = 1;                        // vita.cfg: audio=0 keeps the game muted (60 fps fallback)
static int   g_msaa = 0;                         // 0, 2 or 4 (vita.cfg: msaa=)
static int   g_calib_valid = 0;
static float g_n_roll = 0.f, g_n_pitch = 0.f;
static float g_kx = 0.f, g_ky = 0.f;                 // lerped stick/dpad
static float g_u[3] = { 0.f, -0.2955f, 0.955f };     // raw game-frame vector (g)
static float g_f[3] = { 0.f, -0.2955f, 0.955f };     // filtered
static uint64_t g_accel_last_us = 0;
static uint64_t g_jump_until_us = 0;
static uint32_t g_accel_prev_btn = 0;

static void save_cfg(void) {
    FILE *f = fopen("ux0:data/BounceEvolution/vita.cfg", "w");
    if (!f) return;
    fprintf(f, "cfg_ver=3\naudio=%d\nroll_sign=%d\npitch_sign=%d\ntilt_gain=%.2f\nmsaa=%d\n", g_audio, g_roll_sign < 0 ? -1 : 1, g_pitch_sign < 0 ? -1 : 1, g_tilt_gain, g_msaa);
    fclose(f);
}
static void load_cfg(void) {
    FILE *f = fopen("ux0:data/BounceEvolution/vita.cfg", "r");
    if (!f) { save_cfg(); return; }
    char line[96];
    int ver = 1;
    while (fgets(line, sizeof(line), f)) {
        int iv; float fv;
        if (sscanf(line, "cfg_ver=%d", &iv) == 1) ver = iv;
        else if (sscanf(line, "audio=%d", &iv) == 1) g_audio = iv ? 1 : 0;
        else if (sscanf(line, "roll_sign=%d", &iv) == 1) g_roll_sign = iv < 0 ? -1.f : 1.f;
        else if (sscanf(line, "pitch_sign=%d", &iv) == 1) g_pitch_sign = iv < 0 ? -1.f : 1.f;
        else if (sscanf(line, "tilt_gain=%f", &fv) == 1 && fv > 0.1f && fv < 8.f) g_tilt_gain = fv;
        else if (sscanf(line, "msaa=%d", &iv) == 1 && (iv == 0 || iv == 2 || iv == 4)) g_msaa = iv;
    }
    fclose(f);
    if (ver < 3) { g_msaa = 0; save_cfg(); }     // old cfgs carried msaa=2 from an earlier default
}

static void tilt_angles(const SceMotionState *ms, float *roll, float *pitch) {
    float ax = ms->acceleration.x, ay = ms->acceleration.y, az = ms->acceleration.z;
    *roll  = atan2f(ax, sqrtf(ay * ay + az * az));
    *pitch = atan2f(ay, sqrtf(ax * ax + az * az));
}

static void accel_step(void) {
    SceCtrlData pad;
    sceCtrlPeekBufferPositive(0, &pad, 1);
    uint64_t now = sceKernelGetProcessTimeWide();

    // --- button handling ---------------------------------------------------
    uint32_t pressed = pad.buttons & ~g_accel_prev_btn;
    g_accel_prev_btn = pad.buttons;
    if (pressed & SCE_CTRL_CROSS) g_jump_until_us = now + 100000;
    if (pad.buttons & SCE_CTRL_SELECT) {
        if (pressed & SCE_CTRL_LTRIGGER) { g_roll_sign  = -g_roll_sign;  save_cfg(); vita_log("[ACCEL] roll_sign=%d",  (int)g_roll_sign); }
        else if (pressed & SCE_CTRL_RTRIGGER) { g_pitch_sign = -g_pitch_sign; save_cfg(); vita_log("[ACCEL] pitch_sign=%d", (int)g_pitch_sign); }
        else if (pressed & SCE_CTRL_UP)   { g_tilt_gain += 0.25f; if (g_tilt_gain > 4.f)   g_tilt_gain = 4.f;   save_cfg(); vita_log("[ACCEL] tilt_gain=%.2f", g_tilt_gain); }
        else if (pressed & SCE_CTRL_DOWN) { g_tilt_gain -= 0.25f; if (g_tilt_gain < 0.25f) g_tilt_gain = 0.25f; save_cfg(); vita_log("[ACCEL] tilt_gain=%.2f", g_tilt_gain); }
        else if (pressed & SCE_CTRL_SELECT) { g_calib_valid = 0; }
    } else if (pressed & SCE_CTRL_SELECT) {
        g_calib_valid = 0;
    }

    // --- stick / dpad -> lerped -1..1 (as in Maemulator) ---------------------
    float tx = ((int)pad.lx - 128) / 128.0f, ty = ((int)pad.ly - 128) / 128.0f;
    if (fabsf(tx) < 0.15f) tx = 0.f;
    if (fabsf(ty) < 0.15f) ty = 0.f;
    if (pad.buttons & SCE_CTRL_LEFT)  tx = -1.f;
    if (pad.buttons & SCE_CTRL_RIGHT) tx =  1.f;
    if (pad.buttons & SCE_CTRL_UP)    ty = -1.f;
    if (pad.buttons & SCE_CTRL_DOWN)  ty =  1.f;
    if (pad.buttons & SCE_CTRL_SELECT) { tx = 0.f; ty = 0.f; }      // SELECT+dpad is a settings combo
    g_kx = (tx != 0.f) ? g_kx * 0.7f + tx * 0.3f : g_kx * 0.3f;
    g_ky = (ty != 0.f) ? g_ky * 0.7f + ty * 0.3f : g_ky * 0.3f;

    // --- real tilt -----------------------------------------------------------
    float droll = 0.f, dpitch = 0.f;
    SceMotionState ms;
    if (sceMotionGetState(&ms) >= 0) {
        float r, p;
        tilt_angles(&ms, &r, &p);
        if (!g_calib_valid) { g_n_roll = r; g_n_pitch = p; g_calib_valid = 1; vita_log("[ACCEL] calibrated neutral roll=%.3f pitch=%.3f", r, p); }
        float dr = r - g_n_roll, dp = p - g_n_pitch;
        const float dz = 0.02f;                                      // ~1 degree dead zone, removes sensor jitter
        dr = fabsf(dr) < dz ? 0.f : dr - (dr > 0 ? dz : -dz);
        dp = fabsf(dp) < dz ? 0.f : dp - (dp > 0 ? dz : -dz);
        droll  = dr * g_roll_sign  * g_tilt_gain;
        dpitch = dp * g_pitch_sign * g_tilt_gain;
        static int dbg = 0;
        if (g_verbose && (++dbg % 60) == 0)
            vita_log("[ACCEL] g=(%.2f %.2f %.2f) roll=%.3f pitch=%.3f droll=%.3f dpitch=%.3f", ms.acceleration.x, ms.acceleration.y, ms.acceleration.z, r, p, droll, dpitch);
    }

    // --- Maemulator rotation of the rest vector (0,0,-1) ----------------------
    float rx = g_kx * 0.8f + droll;
    float ry = g_ky + 0.3f + dpitch;              // +0.3: "slightly tilted towards the player"
    // same extreme range the keyboard model reaches (rx +-0.8, ry -0.7..1.3) so the ball can never get more force than on N900/N9
    if (rx >  0.9f) rx =  0.9f;  if (rx < -0.9f) rx = -0.9f;
    if (ry >  1.3f) ry =  1.3f;  if (ry < -0.7f) ry = -0.7f;
    float sy = sinf(-ry), cy = cosf(-ry), sx = sinf(rx), cx = cosf(rx);
    float xx = 0.f, yy = 0.f, zz = -1.f;
    float yz = sy * yy + cy * zz;
    float vx = cx * xx + sx * yz;
    float vy = cy * yy + -sy * zz;
    float vz = -sx * xx + cx * yz;
    if (now < g_jump_until_us) vz += -30.f;

    // game-side conversion of the sysfs values: (-x, +y, -z)
    g_u[0] = -vx; g_u[1] = vy; g_u[2] = -vz;
    for (int i = 0; i < 3; i++) g_f[i] = g_f[i] * 0.8f + g_u[i] * 0.2f;
}

static void accel_tick(void) {
    uint64_t now = sceKernelGetProcessTimeWide();
    if (g_accel_last_us == 0) g_accel_last_us = now - 33000;
    int steps = 0;
    while (now - g_accel_last_us >= 33000 && steps < 4) { accel_step(); g_accel_last_us += 33000; steps++; }
    if (now - g_accel_last_us >= 33000) g_accel_last_us = now;   // drop backlog
}

// Called from the patched Accelerometer::getData / getDataFiltered (softfp ABI,
// hidden return pointer in r0, `this` in r1)
SOFTFP float *__wrap_vita_accel_get_data_raw(float *out_vec3, void *accel_this) {
    accel_tick();
    out_vec3[0] = g_u[0]; out_vec3[1] = g_u[1]; out_vec3[2] = g_u[2];
    return out_vec3;
}
SOFTFP float *__wrap_vita_accel_get_data_filtered(float *out_vec3, void *accel_this) {
    accel_tick();
    out_vec3[0] = g_f[0]; out_vec3[1] = g_f[1]; out_vec3[2] = g_f[2];
    return out_vec3;
}

// ============================================================================
// 3. X11 Window & Touch / Button Event Translation Layer
// ============================================================================
typedef struct FakeScreen {
    uint32_t ext_data;     // +0x00
    uint32_t display;      // +0x04
    uint32_t root;         // +0x08 (Root Window ID read at 0xc2f54)
    uint32_t width;        // +0x0c
    uint32_t height;       // +0x10
    uint32_t mwidth;       // +0x14
    uint32_t mheight;      // +0x18
    uint32_t ndepths;      // +0x1c
    uint32_t depths;       // +0x20
    uint32_t root_depth;   // +0x24 (Root depth read at 0xc2f5c)
    uint32_t pad[16];
} FakeScreen;

typedef struct FakeDisplay {
    uint8_t pad0[0x8c];
    FakeScreen *screens;   // +0x8c (Read at 0xc2f44)
    uint32_t pad1[16];
} FakeDisplay;

static FakeScreen g_screen = { 0, 0, 0x1001, 800, 480, 800, 480, 1, 0, 24, {0} };
static FakeDisplay g_display;

typedef struct FakeXEvent {
    int32_t type;          // +0x00 (2=KeyPress, 3=KeyRelease, 4=ButtonPress, 5=ButtonRelease, 6=MotionNotify)
    uint32_t serial;       // +0x04
    int32_t send_event;    // +0x08
    void *display;         // +0x0c
    uint32_t window;       // +0x10
    uint32_t root;         // +0x14
    uint32_t subwindow;    // +0x18
    uint32_t time;         // +0x1c
    int32_t x;             // +0x20 (Read at 0xc1890 / 0xc18d8 / 0xc1930)
    int32_t y;             // +0x24 (Read at 0xc1898 / 0xc18dc / 0xc1934)
    int32_t x_root;        // +0x28
    int32_t y_root;        // +0x2c
    uint32_t state;        // +0x30
    uint32_t button;       // +0x34 (Read at 0xc18b8 / 0xc1910: 1=Left button, or KeySym for KeyPress)
    uint32_t pad[16];
} FakeXEvent;

static FakeXEvent g_ev_queue[32];
static int g_ev_head = 0, g_ev_tail = 0;
static int g_prev_touch = 0;
static int g_last_tx = 400, g_last_ty = 240;
static uint32_t g_prev_buttons = 0;

static void push_xevent(int type, int x, int y, uint32_t button) {
    int next = (g_ev_tail + 1) & 31;
    if (next == g_ev_head) return;
    memset(&g_ev_queue[g_ev_tail], 0, sizeof(FakeXEvent));
    g_ev_queue[g_ev_tail].type = type;
    g_ev_queue[g_ev_tail].display = &g_display;
    // Deliver to the window id the game stored (falls back to our fake id)
    g_ev_queue[g_ev_tail].window = *(uint32_t *)(bounce_x11_state + 4) ? *(uint32_t *)(bounce_x11_state + 4) : 0x2002;
    g_ev_queue[g_ev_tail].root = 0x1001;
    g_ev_queue[g_ev_tail].x = x;
    g_ev_queue[g_ev_tail].y = y;
    // Pointer events always use Button1; key events carry their KeySym
    g_ev_queue[g_ev_tail].button = (type == 2 || type == 3) ? button : 1;
    g_ev_tail = next;
}

static int g_exposed = 0;
static void poll_vita_inputs(void) {
    if (!g_exposed) { g_exposed = 1; push_xevent(19 /* MapNotify */, 0, 0, 0); push_xevent(12 /* Expose */, 0, 0, 0); }
    // Keep window focus flags active so flushMessages never stalls
    bounce_x11_state[12] = 0;
    bounce_x11_state[13] = 1;

    SceTouchData touch;
    sceTouchPeek(SCE_TOUCH_PORT_FRONT, &touch, 1);
    int down = (touch.reportNum > 0);
    if (down) {
        int tx = (touch.report[0].x * 800) / 1920;
        int ty = (touch.report[0].y * 480) / 1088;
        if (!g_prev_touch) {
            push_xevent(4 /* ButtonPress */, tx, ty, 1);
        } else if (tx != g_last_tx || ty != g_last_ty) {
            push_xevent(6 /* MotionNotify */, tx, ty, 1);
        }
        g_last_tx = tx;
        g_last_ty = ty;
    } else if (g_prev_touch) {
        push_xevent(5 /* ButtonRelease */, g_last_tx, g_last_ty, 1);
    }
    g_prev_touch = down;

    // Start -> Escape. (Jump is handled by the accelerometer model: Cross = spike on Z.)
    SceCtrlData pad;
    sceCtrlPeekBufferPositive(0, &pad, 1);
    uint32_t changed = pad.buttons ^ g_prev_buttons;
    if (changed & SCE_CTRL_START) {
        push_xevent((pad.buttons & SCE_CTRL_START) ? 2 : 3, 0, 0, 0xff1b /* XK_Escape */);
    }
    g_prev_buttons = pad.buttons;
}

void *__wrap_XOpenDisplay(const char *name) {
    memset(&g_display, 0, sizeof(g_display));
    g_display.screens = &g_screen;
    return &g_display;
}
int __wrap_XCloseDisplay(void *dpy) { return 0; }
int __wrap_XDefaultScreen(void *dpy) { return 0; }
int __wrap_XMatchVisualInfo(void *dpy, int screen, int depth, int c_class, void *vinfo_ret) { return 1; }
uint32_t __wrap_XCreateColormap(void *dpy, uint32_t w, void *vis, int alloc) { return 1; }
int __wrap_XFreeColormap(void *dpy, uint32_t cmap) { return 0; }
int __wrap_XSelectInput(void *dpy, uint32_t w, long mask) { return 0; }
uint32_t __wrap_XCreateWindow(void *dpy, uint32_t parent, int x, int y, unsigned int w, unsigned int h,
                              unsigned int bw, int depth, unsigned int c_class, void *vis, unsigned long vmask, void *attr) {
    return 0x2002;
}
int __wrap_XDestroyWindow(void *dpy, uint32_t w) { return 0; }
int __wrap_XMapWindow(void *dpy, uint32_t w) { return 0; }
uint32_t __wrap_XInternAtom(void *dpy, const char *name, int only_if_exists) {
    if (!name) return 0;
    if (strcmp(name, "_MB_CURRENT_APP_WINDOW") == 0) return 100;
    if (strcmp(name, "_NET_ACTIVE_WINDOW") == 0) return 101;
    if (strcmp(name, "WM_PROTOCOLS") == 0) return 102;
    if (strcmp(name, "WM_DELETE_WINDOW") == 0) return 103;
    return 104;
}
int __wrap_XChangeProperty(void *dpy, uint32_t w, uint32_t prop, uint32_t type, int fmt, int mode, const unsigned char *data, int nelements) { return 0; }
void *__wrap_XAllocClassHint(void) { return calloc(1, 16); }
int __wrap_XSetClassHint(void *dpy, uint32_t w, void *hint) { return 0; }
int __wrap_XSetWMHints(void *dpy, uint32_t w, void *wmhints) { return 0; }
int __wrap_XSetWindowBackground(void *dpy, uint32_t w, unsigned long bg) { return 0; }
int __wrap_XIfEvent(void *dpy, void *event_ret, int (*pred)(void*, void*, char*), char *arg) { return 0; }
int __wrap_XSetWMProtocols(void *dpy, uint32_t w, uint32_t *protocols, int count) { return 1; }
int __wrap_XFlush(void *dpy) { return 0; }
int __wrap_XFree(void *data) { if (data) free(data); return 0; }
int __wrap_XSendEvent(void *dpy, uint32_t w, int prop, long mask, void *ev) { return 1; }
int __wrap_XGetGeometry(void *dpy, uint32_t d, uint32_t *root, int *x, int *y, unsigned int *w, unsigned int *h, unsigned int *bw, unsigned int *depth) {
    if (x) *x = 0; if (y) *y = 0; if (w) *w = 800; if (h) *h = 480; if (bw) *bw = 0; if (depth) *depth = 24;
    return 1;
}
int __wrap_XGetWindowProperty(void *dpy, uint32_t w, uint32_t prop, long off, long len, int del, uint32_t req_type,
                              uint32_t *actual_type, int *actual_fmt, unsigned long *nitems, unsigned long *bytes_after, unsigned char **prop_ret) {
    if (actual_type) *actual_type = 33; // XA_WINDOW (checked at 0xc1494)
    if (actual_fmt)  *actual_fmt  = 32; // 32-bit format (checked at 0xc15e4)
    if (nitems)      *nitems      = 1;  // 1 item (checked at 0xc15f0)
    if (bytes_after) *bytes_after = 0;
    if (prop_ret) {
        uint32_t *win_ptr = (uint32_t *)malloc(sizeof(uint32_t));
        *win_ptr = 0x2002; // Match our window ID at bounce_x11_state + 4
        *prop_ret = (unsigned char *)win_ptr;
    }
    return 0; // Success
}
int __wrap_XPending(void *dpy) {
    poll_vita_inputs();
    return (g_ev_head != g_ev_tail) ? 1 : 0;
}
int __wrap_XNextEvent(void *dpy, FakeXEvent *ev) {
    poll_vita_inputs();
    if (g_ev_head != g_ev_tail) {
        if (ev) *ev = g_ev_queue[g_ev_head];
        g_ev_head = (g_ev_head + 1) & 31;
    } else {
        sceKernelDelayThread(16000);
        if (ev) memset(ev, 0, sizeof(FakeXEvent));
    }
    return 0;
}
int __wrap_XPeekEvent(void *dpy, FakeXEvent *ev) {
    poll_vita_inputs();
    if (g_ev_head != g_ev_tail && ev) *ev = g_ev_queue[g_ev_head];
    return 0;
}
uint32_t __wrap_XLookupKeysym(FakeXEvent *ev, int idx) {
    return ev ? ev->button : 0;
}

// ============================================================================
// 4. D-Bus Stubs (Must return Primary Owner = 1 in dbus_bus_request_name)
// ============================================================================
void __wrap_dbus_error_init(void *err) { if (err) memset(err, 0, 32); }
void __wrap_dbus_error_free(void *err) { if (err) memset(err, 0, 32); }
int  __wrap_dbus_error_is_set(const void *err) { return 0; }
void *__wrap_dbus_bus_get(int type, void *err) { return (void *)0xDB050001; }
int  __wrap_dbus_bus_request_name(void *conn, const char *name, unsigned int flags, void *err) {
    return 1; // DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER (checked at 0xc37c0 & 0xc380c)
}
void __wrap_dbus_bus_add_match(void *conn, const char *rule, void *err) {}
int  __wrap_dbus_connection_add_filter(void *conn, void *func, void *user_data, void *free_func) { return 1; }
void __wrap_dbus_connection_flush(void *conn) {}
int  __wrap_dbus_connection_read_write_dispatch(void *conn, int timeout_ms) { return 1; }
int  __wrap_dbus_connection_send(void *conn, void *msg, uint32_t *serial) { return 1; }
void __wrap_dbus_connection_unref(void *conn) {}
int  __wrap_dbus_message_is_signal(void *msg, const char *iface, const char *member) { return 0; }
int  __wrap_dbus_message_iter_init(void *msg, void *iter) { return 0; }
void __wrap_dbus_message_iter_get_basic(void *iter, void *val) {}
void *__wrap_dbus_message_new_signal(const char *path, const char *iface, const char *name) { return (void *)0xDB050002; }
void __wrap_dbus_message_unref(void *msg) {}

// ============================================================================
// 5. PulseAudio -> PS Vita SceAudioOut Translation Layer
// ============================================================================
typedef void (*pa_ctx_cb_t)(void *ctx, void *userdata);
typedef void (*pa_stm_cb_t)(void *stm, void *userdata);
typedef void (*pa_wrt_cb_t)(void *stm, size_t nbytes, void *userdata);

typedef struct pa_sample_spec_n900 {
    int format;
    uint32_t rate;
    uint8_t channels;
} pa_sample_spec_n900;

// pa_mainloop_lock() can be re-entered from inside the write callback -> recursive mutex
static pthread_mutex_t g_pa_mutex = PTHREAD_MUTEX_INITIALIZER;
static void pa_mutex_make_recursive(void) {
    pthread_mutexattr_t a;
    pthread_mutexattr_init(&a);
    pthread_mutexattr_settype(&a, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&g_pa_mutex, &a);
    pthread_mutexattr_destroy(&a);
}
static pa_ctx_cb_t g_ctx_cb = NULL;
static void *g_ctx_ud = NULL;
static pa_stm_cb_t g_stm_cb = NULL;
static void *g_stm_ud = NULL;
static pa_wrt_cb_t g_wrt_cb = NULL;
static void *g_wrt_ud = NULL;
static volatile int g_pa_running = 0;
static volatile int g_pa_gen = 0;           // bumped on every connect so a stale audio thread exits instead of doubling the mixing
static pthread_t g_pa_thread;
static uint32_t g_pa_rate = 44100;
static uint8_t  g_pa_channels = 2;
static int g_pa_format = 3;                 // PA_SAMPLE_S16LE
static int g_vita_audio_port = -1;

// Byte ring between the game's write() calls and the Vita audio grain output
#define AUD_RING_SZ 65536
static uint8_t  g_aring[AUD_RING_SZ];
static uint32_t g_ar_r = 0, g_ar_w = 0;      // monotonically increasing byte counters
static pthread_mutex_t g_ar_mx = PTHREAD_MUTEX_INITIALIZER;
static uint32_t g_cb_max_us = 0, g_cb_sum_us = 0, g_cb_n = 0;      // audio callback timing (printed only in verbose mode)
static uint32_t g_aud_cb_calls = 0, g_aud_bytes_in = 0, g_aud_overflows = 0, g_aud_grains = 0, g_aud_silent = 0;

static uint32_t ar_fill(void) { return g_ar_w - g_ar_r; }
static void ar_push(const uint8_t *d, uint32_t n) {
    pthread_mutex_lock(&g_ar_mx);
    if (n > AUD_RING_SZ - ar_fill()) { g_aud_overflows++; n = AUD_RING_SZ - ar_fill(); }
    for (uint32_t i = 0; i < n; i++) g_aring[(g_ar_w + i) & (AUD_RING_SZ - 1)] = d[i];
    g_ar_w += n;
    pthread_mutex_unlock(&g_ar_mx);
}
static uint32_t ar_pop(uint8_t *d, uint32_t n) {
    pthread_mutex_lock(&g_ar_mx);
    uint32_t avail = ar_fill();
    if (n > avail) n = avail;
    for (uint32_t i = 0; i < n; i++) d[i] = g_aring[(g_ar_r + i) & (AUD_RING_SZ - 1)];
    g_ar_r += n;
    pthread_mutex_unlock(&g_ar_mx);
    return n;
}

void *__wrap_pa_threaded_mainloop_new(void) { return (void *)0xFA000001; }
void  __wrap_pa_threaded_mainloop_free(void *m) {}
void *__wrap_pa_threaded_mainloop_get_api(void *m) { return (void *)0xFA000002; }
void  __wrap_pa_threaded_mainloop_lock(void *m) { pthread_mutex_lock(&g_pa_mutex); }
void  __wrap_pa_threaded_mainloop_unlock(void *m) { pthread_mutex_unlock(&g_pa_mutex); }

void *__wrap_pa_context_new(void *api, const char *name) { return (void *)0xFA000003; }
void  __wrap_pa_context_unref(void *c) {}
int   __wrap_pa_context_errno(void *c) { return 0; }
const char *__wrap_pa_strerror(int error) { return "OK"; }
void  __wrap_pa_context_set_state_callback(void *c, pa_ctx_cb_t cb, void *userdata) {
    g_ctx_cb = cb;
    g_ctx_ud = userdata;
}
int   __wrap_pa_context_connect(void *c, const char *server, int flags, const void *api) { return 0; }
void  __wrap_pa_context_disconnect(void *c) {}
int   __wrap_pa_context_get_state(void *c) { return 4; /* PA_CONTEXT_READY */ }

int   __wrap_pa_threaded_mainloop_start(void *m) {
    if (g_ctx_cb) g_ctx_cb((void *)0xFA000003, g_ctx_ud);
    return 0;
}
void  __wrap_pa_threaded_mainloop_stop(void *m) {
    g_pa_running = 0;
}
void *__wrap_pa_channel_map_init_auto(void *map, unsigned int ch, int def) { return map; }
void *__wrap_pa_stream_new(void *c, const char *name, const pa_sample_spec_n900 *ss, const void *map) {
    if (ss) {
        g_pa_format   = ss->format;
        g_pa_rate     = ss->rate ? ss->rate : 44100;
        g_pa_channels = ss->channels ? ss->channels : 2;
    }
    vita_log("[PA] stream_new format=%d rate=%u channels=%u", g_pa_format, (unsigned)g_pa_rate, (unsigned)g_pa_channels);
    return (void *)0xFA000004;
}
void  __wrap_pa_stream_unref(void *s) {}
void  __wrap_pa_stream_set_state_callback(void *s, pa_stm_cb_t cb, void *userdata) {
    g_stm_cb = cb;
    g_stm_ud = userdata;
}
void  __wrap_pa_stream_set_write_callback(void *s, pa_wrt_cb_t cb, void *userdata) {
    g_wrt_cb = cb;
    g_wrt_ud = userdata;
    vita_log("[PA] write callback registered");
}
int   __wrap_pa_stream_get_state(void *s) { return 2; /* PA_STREAM_READY */ }

static int pa_rate_ok(uint32_t r) {
    static const uint32_t ok[] = { 8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000 };
    for (unsigned i = 0; i < sizeof(ok) / sizeof(ok[0]); i++) if (ok[i] == r) return 1;
    return 0;
}

#define AUD_GRAIN 1024
#define AUD_REQUEST_BYTES 4096            // smaller mixing bursts than the Maemulator's 8192: shorter stalls under the PA lock
static void *pa_audio_thread(void *arg) {
    int my_gen = (int)(intptr_t)arg;
    uint32_t rate = g_pa_rate;
    if (!pa_rate_ok(rate)) { vita_log("[PA] unsupported rate %u, falling back to 48000", (unsigned)rate); rate = 48000; }
    SceAudioOutMode mode = (g_pa_channels == 1) ? SCE_AUDIO_OUT_MODE_MONO : SCE_AUDIO_OUT_MODE_STEREO;
    g_vita_audio_port = sceAudioOutOpenPort(SCE_AUDIO_OUT_PORT_TYPE_MAIN, AUD_GRAIN, rate, mode);
    vita_log("[PA] sceAudioOutOpenPort(MAIN, %d, %u, %s) = 0x%08x", AUD_GRAIN, (unsigned)rate,
             g_pa_channels == 1 ? "mono" : "stereo", (unsigned)g_vita_audio_port);
    if (g_vita_audio_port < 0) { g_pa_running = 0; return NULL; }

    uint32_t frame_bytes = (g_pa_channels == 1 ? 1 : 2) * sizeof(int16_t);
    uint32_t grain_bytes = AUD_GRAIN * frame_bytes;
    static int16_t grain[AUD_GRAIN * 2];
    uint64_t last_stat = sceKernelGetProcessTimeWide();

    while (g_pa_running && my_gen == g_pa_gen) {
        // Ask the game for more PCM until we hold two grains (or it has nothing to give)
        for (int guard = 0; guard < 8 && ar_fill() < grain_bytes * 2 && g_wrt_cb; guard++) {
            uint32_t before = g_ar_w;
            pthread_mutex_lock(&g_pa_mutex);
            uint64_t tb = sceKernelGetProcessTimeWide();
            g_wrt_cb((void *)0xFA000004, AUD_REQUEST_BYTES, g_wrt_ud);
            uint32_t run = (uint32_t)(sceKernelGetProcessTimeWide() - tb);
            pthread_mutex_unlock(&g_pa_mutex);
            g_cb_n++; g_cb_sum_us += run; if (run > g_cb_max_us) g_cb_max_us = run;
            g_aud_cb_calls++;
            if (g_ar_w == before) break;
        }
        uint32_t got = ar_pop((uint8_t *)grain, grain_bytes);
        if (got < grain_bytes) memset((uint8_t *)grain + got, 0, grain_bytes - got);   // pad with silence
        if (got == 0) g_aud_silent++;
        sceAudioOutOutput(g_vita_audio_port, grain);                                   // blocks for one grain
        g_aud_grains++;

        uint64_t now = sceKernelGetProcessTimeWide();
        if (now - last_stat > 10000000) {
            last_stat = now;
            vlog("[PA] stats: callbacks=%u bytes_in=%u grains=%u silent_grains=%u overflows=%u | mix avg %u us max %u us",
                 (unsigned)g_aud_cb_calls, (unsigned)g_aud_bytes_in, (unsigned)g_aud_grains,
                 (unsigned)g_aud_silent, (unsigned)g_aud_overflows, g_cb_n ? g_cb_sum_us / g_cb_n : 0, g_cb_max_us);
            g_cb_sum_us = g_cb_max_us = g_cb_n = 0;
        }
    }
    sceAudioOutReleasePort(g_vita_audio_port);
    g_vita_audio_port = -1;
    return NULL;
}

int   __wrap_pa_stream_connect_playback(void *s, const char *dev, const void *attr, int flags, const void *vol, void *sync) {
    vita_log("[PA] connect_playback");
    if (g_stm_cb) g_stm_cb((void *)0xFA000004, g_stm_ud);
    g_pa_gen++;
    g_pa_running = 1;
    pthread_create(&g_pa_thread, NULL, pa_audio_thread, (void *)(intptr_t)g_pa_gen);
    return 0;
}
int   __wrap_pa_stream_disconnect(void *s) {
    g_pa_running = 0;
    return 0;
}
int   __wrap_pa_stream_write(void *s, const void *data, size_t nbytes, void (*free_cb)(void*), int64_t offset, int seek) {
    if (data && nbytes > 0) {
        g_aud_bytes_in += (uint32_t)nbytes;
        if (g_pa_format == 3) {                                   // S16LE: copy as is
            ar_push((const uint8_t *)data, (uint32_t)nbytes);
        } else if (g_pa_format == 5) {                            // FLOAT32LE -> S16
            const float *in = (const float *)data;
            size_t n = nbytes / 4;
            static int16_t tmp[4096];
            while (n) {
                size_t c = n > 4096 ? 4096 : n;
                for (size_t i = 0; i < c; i++) {
                    float v = in[i] * 32767.0f;
                    tmp[i] = (int16_t)(v > 32767.f ? 32767 : (v < -32768.f ? -32768 : v));
                }
                ar_push((const uint8_t *)tmp, (uint32_t)(c * 2));
                in += c; n -= c;
            }
        }
    }
    if (free_cb) free_cb((void *)data);                           // we copied the data; release the game's buffer
    return 0;
}

// The game starts muted unless bounce.conf says otherwise (reBounce for the N9 un-mutes it
// by posting these four events to GameFlowManager::postEvent at 0x54948).
static int g_unmuted = 0;
static void try_unmute_audio(void) {
    if (g_unmuted || !g_audio) return;
    void *app = *(void **)(bounce_rw_base + (0x001d14b0 - 0x001cf000));
    if (!app) return;
    void *gfm = *(void **)((uint8_t *)app + 0x2b0);
    if (!gfm) return;
    typedef void (*post_event_fn)(void *self, int event_id, int param1, void *param2);
    post_event_fn post = (post_event_fn)(bounce_rx_base + (0x00054948 - 0x00008000));
    int32_t empty_string = -1;
    post(gfm, 7,  -1, &empty_string);   // add menu scene
    post(gfm, 13, -1, &empty_string);   // go to pause menu
    post(gfm, 38,  1, &empty_string);   // change audio status -> on
    post(gfm, 23, -1, &empty_string);   // return to game
    g_unmuted = 1;
    vita_log("[AUDIO] un-mute events posted (app=%p gfm=%p)", app, gfm);
}

// ============================================================================
// 6. EGL + OpenGL ES 2.0 -> vitaGL Translation Layer (with SoftFP Thunks)
// ============================================================================
void *__wrap_eglGetDisplay(int display_id) { return (void *)1; }
int   __wrap_eglInitialize(void *dpy, int *major, int *minor) {
    if (major) *major = 1;
    if (minor) *minor = 4;
    return 1;
}
int   __wrap_eglGetConfigs(void *dpy, void **configs, int config_size, int *num_config) {
    if (num_config) *num_config = 1;
    if (configs && config_size > 0) configs[0] = (void *)1;
    return 1;
}
int   __wrap_eglChooseConfig(void *dpy, const int *attrib_list, void **configs, int config_size, int *num_config) {
    if (num_config) *num_config = 1;
    if (configs && config_size > 0) configs[0] = (void *)1;
    return 1;
}
void *__wrap_eglCreateWindowSurface(void *dpy, void *config, uint32_t win, const int *attrib_list) { return (void *)1; }
void *__wrap_eglCreateContext(void *dpy, void *config, void *share_context, const int *attrib_list) { return (void *)1; }
int   __wrap_eglMakeCurrent(void *dpy, void *draw, void *read, void *ctx) { return 1; }
static uint32_t g_frames = 0;
static uint64_t g_stat_t0 = 0;
static uint64_t g_last_swap_us = 0;
#define FRAME_CAP_US 25000
static volatile uint64_t g_time_debt_us = 0;
static uint64_t g_last_game_us = 0;
int   __wrap_eglSwapBuffers(void *dpy, void *surface) {
    vglSwapBuffers(GL_FALSE);
    g_frames++;
    {
        uint64_t t = sceKernelGetProcessTimeWide();
        if (g_last_swap_us && t - g_last_swap_us > FRAME_CAP_US) g_time_debt_us += (t - g_last_swap_us) - 16667;
        g_last_swap_us = t;
    }
    if (g_frames == 1) vlog("[FRAME] first eglSwapBuffers (loading screen visible)");
    try_unmute_audio();
    if ((g_frames % 300) == 0) {
        uint64_t now = sceKernelGetProcessTimeWide();
        if (g_stat_t0) vlog("[FRAME] %u frames, %.1f fps over last 300", (unsigned)g_frames, 300.0 * 1e6 / (double)(now - g_stat_t0));
        g_stat_t0 = now;
    }
    return 1;
}
int   __wrap_eglDestroySurface(void *dpy, void *surface) { return 1; }
int   __wrap_eglDestroyContext(void *dpy, void *ctx) { return 1; }
int   __wrap_eglTerminate(void *dpy) { return 1; }
int   __wrap_eglGetError(void) { return 0x3000; /* EGL_SUCCESS */ }

// Float-taking GLES2 functions (softfp -> hardfp)
SOFTFP void __wrap_glClearColor(GLclampf r, GLclampf g, GLclampf b, GLclampf a) { glClearColor(r, g, b, a); }
SOFTFP void __wrap_glUniform1f(GLint loc, GLfloat v0) { glUniform1f(loc, v0); }
SOFTFP void __wrap_glUniform4f(GLint loc, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3) { glUniform4f(loc, v0, v1, v2, v3); }

// Non-float-register GLES2 functions
void __wrap_glActiveTexture(GLenum tex) { glActiveTexture(tex); }
void __wrap_glAttachShader(GLuint prog, GLuint sh) { vlog("[ATTACH] prog=%u sh=%u", (unsigned)prog, (unsigned)sh); glAttachShader(prog, sh); }
void __wrap_glBindBuffer(GLenum target, GLuint buf) { glBindBuffer(target, buf); }
void __wrap_glBindFramebuffer(GLenum target, GLuint fb) { glBindFramebuffer(target, fb); }
void __wrap_glBindRenderbuffer(GLenum target, GLuint rb) { glBindRenderbuffer(target, rb); }
void __wrap_glBindTexture(GLenum target, GLuint tex) { glBindTexture(target, tex); }
void __wrap_glBlendEquation(GLenum mode) { glBlendEquation(mode); }
void __wrap_glBlendFunc(GLenum sfactor, GLenum dfactor) { glBlendFunc(sfactor, dfactor); }
void __wrap_glBufferData(GLenum target, GLsizeiptr size, const void *data, GLenum usage) { glBufferData(target, size, data, usage); }
GLenum __wrap_glCheckFramebufferStatus(GLenum target) { return glCheckFramebufferStatus(target); }
void __wrap_glClear(GLbitfield mask) { glClear(mask); }

void __wrap_glCompressedTexImage2D(GLenum target, GLint level, GLenum internalformat, GLsizei w, GLsizei h, GLint border, GLsizei imageSize, const void *data) {
    glCompressedTexImage2D(target, level, internalformat, w, h, border, imageSize, data);
}
GLuint __wrap_glCreateProgram(void) { return glCreateProgram(); }
GLuint __wrap_glCreateShader(GLenum type) { return glCreateShader(type); }
void __wrap_glCullFace(GLenum mode) { glCullFace(mode); }
void __wrap_glDeleteBuffers(GLsizei n, const GLuint *buffers) { glDeleteBuffers(n, buffers); }
void __wrap_glDeleteFramebuffers(GLsizei n, const GLuint *fbs) { glDeleteFramebuffers(n, fbs); }
void __wrap_glDeleteProgram(GLuint prog) { glDeleteProgram(prog); }
void __wrap_glDeleteRenderbuffers(GLsizei n, const GLuint *rbs) { glDeleteRenderbuffers(n, rbs); }
void __wrap_glDeleteShader(GLuint sh) { glDeleteShader(sh); }
void __wrap_glDeleteTextures(GLsizei n, const GLuint *textures) { glDeleteTextures(n, textures); }
void __wrap_glDepthFunc(GLenum func) { glDepthFunc(func); }
void __wrap_glDepthMask(GLboolean flag) { glDepthMask(flag); }
void __wrap_glDisable(GLenum cap) { glDisable(cap); }
void __wrap_glDisableVertexAttribArray(GLuint index) { glDisableVertexAttribArray(index); }
void __wrap_glDrawArrays(GLenum mode, GLint first, GLsizei count) { glDrawArrays(mode, first, count); }
void __wrap_glDrawElements(GLenum mode, GLsizei count, GLenum type, const void *indices) { glDrawElements(mode, count, type, indices); }
void __wrap_glEnable(GLenum cap) { glEnable(cap); }
void __wrap_glEnableVertexAttribArray(GLuint index) { glEnableVertexAttribArray(index); }
void __wrap_glFramebufferRenderbuffer(GLenum target, GLenum attachment, GLenum rbtarget, GLuint rb) { glFramebufferRenderbuffer(target, attachment, rbtarget, rb); }
void __wrap_glFramebufferTexture2D(GLenum target, GLenum attachment, GLenum textarget, GLuint tex, GLint level) { glFramebufferTexture2D(target, attachment, textarget, tex, level); }
void __wrap_glFrontFace(GLenum mode) { glFrontFace(mode); }
void __wrap_glGenBuffers(GLsizei n, GLuint *buffers) { glGenBuffers(n, buffers); }
void __wrap_glGenFramebuffers(GLsizei n, GLuint *fbs) { glGenFramebuffers(n, fbs); }
void __wrap_glGenRenderbuffers(GLsizei n, GLuint *rbs) { glGenRenderbuffers(n, rbs); }
void __wrap_glGenTextures(GLsizei n, GLuint *textures) { glGenTextures(n, textures); }
void __wrap_glGetActiveUniform(GLuint prog, GLuint index, GLsizei bufSize, GLsizei *length, GLint *size, GLenum *type, GLchar *name) {
    glGetActiveUniform(prog, index, bufSize, length, size, type, name);
}
GLint __wrap_glGetAttribLocation(GLuint prog, const GLchar *name) { return glGetAttribLocation(prog, name); }
void __wrap_glGetIntegerv(GLenum pname, GLint *params) { glGetIntegerv(pname, params); }
void __wrap_glGetProgramInfoLog(GLuint prog, GLsizei bufSize, GLsizei *length, GLchar *infoLog) { glGetProgramInfoLog(prog, bufSize, length, infoLog); }
void __wrap_glGetProgramiv(GLuint prog, GLenum pname, GLint *params) {
    if (pname == 0x8B83 /* GL_VALIDATE_STATUS */) {
        if (params) *params = GL_TRUE;
        return;
    }
    glGetProgramiv(prog, pname, params);
}
void __wrap_glGetShaderInfoLog(GLuint sh, GLsizei bufSize, GLsizei *length, GLchar *infoLog) {
    glGetShaderInfoLog(sh, bufSize, length, infoLog);
    if (infoLog && infoLog[0]) vita_log("[SHADER ERROR] %s", infoLog);
}
void __wrap_glGetShaderiv(GLuint sh, GLenum pname, GLint *params) { glGetShaderiv(sh, pname, params); }
GLint __wrap_glGetUniformLocation(GLuint prog, const GLchar *name) { return glGetUniformLocation(prog, name); }
void __wrap_glGetUniformfv(GLuint prog, GLint loc, GLfloat *params) {
    if (params) params[0] = 0.0f;
}

void __wrap_glPixelStorei(GLenum pname, GLint param) { glPixelStorei(pname, param); }
void __wrap_glRenderbufferStorage(GLenum target, GLenum internalformat, GLsizei w, GLsizei h) { glRenderbufferStorage(target, internalformat, w, h); }
void __wrap_glScissor(GLint x, GLint y, GLsizei w, GLsizei h) { glScissor(x, y, w, h); }

void __wrap_glTexImage2D(GLenum target, GLint level, GLint internalformat, GLsizei w, GLsizei h, GLint border, GLenum format, GLenum type, const void *pixels) {
    glTexImage2D(target, level, internalformat, w, h, border, format, type, pixels);
}
void __wrap_glTexParameteri(GLenum target, GLenum pname, GLint param) { glTexParameteri(target, pname, param); }
void __wrap_glTexSubImage2D(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLsizei w, GLsizei h, GLenum format, GLenum type, const void *pixels) {
    glTexSubImage2D(target, level, xoffset, yoffset, w, h, format, type, pixels);
}
void __wrap_glUniform1i(GLint loc, GLint v0) { glUniform1i(loc, v0); }
void __wrap_glUniformMatrix4fv(GLint loc, GLsizei count, GLboolean transpose, const GLfloat *value) {
    glUniformMatrix4fv(loc, count, transpose, value);
}
void __wrap_glUseProgram(GLuint prog) { glUseProgram(prog); }
void __wrap_glValidateProgram(GLuint prog) {}
void __wrap_glVertexAttribPointer(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void *pointer) {
    glVertexAttribPointer(index, size, type, normalized, stride, pointer);
}
void __wrap_glViewport(GLint x, GLint y, GLsizei w, GLsizei h) {
    if (w == 800 && h == 480) {
        glViewport(0, 0, 960, 544);
    } else {
        glViewport(x, y, w, h);
    }
}

// ============================================================================
// 7. libpng12 -> VitaSDK libpng16 Bridge
// ============================================================================
int __wrap_png_check_sig(png_const_bytep sig, size_t num) { return !png_sig_cmp(sig, 0, num); }
png_structp __wrap_png_create_read_struct(png_const_charp ver, png_voidp err_ptr, png_error_ptr err_fn, png_error_ptr warn_fn) {
    return png_create_read_struct(PNG_LIBPNG_VER_STRING, err_ptr, err_fn, warn_fn);
}
png_infop __wrap_png_create_info_struct(png_const_structrp png_ptr) { return png_create_info_struct(png_ptr); }
void __wrap_png_destroy_read_struct(png_structpp png_ptr_ptr, png_infopp info_ptr_ptr, png_infopp end_info_ptr_ptr) {
    png_destroy_read_struct(png_ptr_ptr, info_ptr_ptr, end_info_ptr_ptr);
}
png_uint_32 __wrap_png_get_IHDR(png_const_structrp png_ptr, png_const_inforp info_ptr, png_uint_32 *w, png_uint_32 *h,
                                int *bit_depth, int *color_type, int *interlace, int *comp, int *filter) {
    return png_get_IHDR(png_ptr, info_ptr, w, h, bit_depth, color_type, interlace, comp, filter);
}
png_uint_32 __wrap_png_get_PLTE(png_const_structrp png_ptr, png_inforp info_ptr, png_colorp *palette, int *num_palette) {
    return png_get_PLTE(png_ptr, info_ptr, palette, num_palette);
}
png_voidp __wrap_png_get_io_ptr(png_const_structrp png_ptr) { return png_get_io_ptr(png_ptr); }
png_uint_32 __wrap_png_get_tRNS(png_const_structrp png_ptr, png_inforp info_ptr, png_bytep *trans_alpha, int *num_trans, png_color_16p *trans_color) {
    return png_get_tRNS(png_ptr, info_ptr, trans_alpha, num_trans, trans_color);
}
png_uint_32 __wrap_png_get_valid(png_const_structrp png_ptr, png_const_inforp info_ptr, png_uint_32 flag) {
    return png_get_valid(png_ptr, info_ptr, flag);
}
void __wrap_png_read_image(png_structrp png_ptr, png_bytepp image) { png_read_image(png_ptr, image); }
void __wrap_png_read_info(png_structrp png_ptr, png_inforp info_ptr) { png_read_info(png_ptr, info_ptr); }
void __wrap_png_set_error_fn(png_structrp png_ptr, png_voidp err_ptr, png_error_ptr err_fn, png_error_ptr warn_fn) {
    png_set_error_fn(png_ptr, err_ptr, err_fn, warn_fn);
}
void __wrap_png_set_gray_1_2_4_to_8(png_structrp png_ptr) { png_set_expand_gray_1_2_4_to_8(png_ptr); }
void __wrap_png_set_read_fn(png_structrp png_ptr, png_voidp io_ptr, png_rw_ptr read_data_fn) {
    png_set_read_fn(png_ptr, io_ptr, read_data_fn);
}
void __wrap_png_set_sig_bytes(png_structrp png_ptr, int num_bytes) { png_set_sig_bytes(png_ptr, num_bytes); }
void __wrap_png_set_tRNS_to_alpha(png_structrp png_ptr) { png_set_tRNS_to_alpha(png_ptr); }

// ============================================================================
// 8. SoftFP Math & Libc/POSIX/Pthread/C++ ABI Wrappers
// ============================================================================
SOFTFP float  __wrap_acosf(float x) { return acosf(x); }
SOFTFP float  __wrap_asinf(float x) { return asinf(x); }
SOFTFP double __wrap_atan2(double y, double x) { return atan2(y, x); }
SOFTFP float  __wrap_atan2f(float y, float x) { return atan2f(y, x); }
SOFTFP float  __wrap_atanf(float x) { return atanf(x); }
SOFTFP float  __wrap_ceilf(float x) { return ceilf(x); }
SOFTFP float  __wrap_cosf(float x) { return cosf(x); }
SOFTFP float  __wrap_coshf(float x) { return coshf(x); }
SOFTFP double __wrap_difftime(time_t t1, time_t t0) { return difftime(t1, t0); }
SOFTFP float  __wrap_expf(float x) { return expf(x); }
SOFTFP float  __wrap_floorf(float x) { return floorf(x); }
SOFTFP double __wrap_fmod(double x, double y) { return fmod(x, y); }
SOFTFP float  __wrap_fmodf(float x, float y) { return fmodf(x, y); }
SOFTFP double __wrap_frexp(double x, int *exp) { return frexp(x, exp); }
SOFTFP double __wrap_ldexp(double x, int exp) { return ldexp(x, exp); }
SOFTFP double __wrap_log(double x) { return log(x); }
SOFTFP float  __wrap_log10f(float x) { return log10f(x); }
SOFTFP float  __wrap_logf(float x) { return logf(x); }
SOFTFP double __wrap_modf(double x, double *iptr) {
    double iv = 0.0;
    double res = modf(x, &iv);
    if (iptr) *iptr = iv;
    return res;
}
SOFTFP double __wrap_pow(double x, double y) { return pow(x, y); }
SOFTFP double __wrap_sin(double x) { return sin(x); }
SOFTFP float  __wrap_sinf(float x) { return sinf(x); }
SOFTFP float  __wrap_sinhf(float x) { return sinhf(x); }
SOFTFP float  __wrap_tanf(float x) { return tanf(x); }
SOFTFP float  __wrap_tanhf(float x) { return tanhf(x); }
SOFTFP double __wrap___strtod_internal(const char *nptr, char **endptr, int group) { return strtod(nptr, endptr); }
long __wrap___strtol_internal(const char *nptr, char **endptr, int base, int group) { return strtol(nptr, endptr, base); }
unsigned long __wrap___strtoul_internal(const char *nptr, char **endptr, int base, int group) { return strtoul(nptr, endptr, base); }
char *__wrap___strdup(const char *s) { return strdup(s); }

// File I/O & glob() with path translation
// The game first looks for textures in bundles/original/ and falls back (via a C++
// exception) to resources/textures/. Resolve that fallback here instead.
static int path_exists(const char *p) { SceIoStat st; return sceIoGetstat(p, &st) >= 0; }
static int asset_fallback(const char *vpath, char *out, size_t sz) {
    const char *base = strrchr(vpath, '/');
    base = base ? base + 1 : vpath;
    const char *ext = strrchr(base, '.');
    if (!ext) return 0;
    static const char *const exts[] = { ".pvr", ".tga", ".hgr", ".prs", ".png", ".dat", ".wav" };
    int ok = 0;
    for (unsigned i = 0; i < sizeof(exts) / sizeof(exts[0]); i++) if (strcasecmp(ext, exts[i]) == 0) ok = 1;
    if (!ok) return 0;
    int is_tga = (strcasecmp(ext, ".tga") == 0);
    static const char *const dirs[] = { "resources/textures", "resources/models", "resources/images",
                                        "resources/particles", "resources/sounds", "resources/musics",
                                        "resources/localization", "bundles/original" };
    int stem = (int)(ext - base);
    for (unsigned i = 0; i < sizeof(dirs) / sizeof(dirs[0]); i++) {
        if (is_tga) { snprintf(out, sz, "ux0:data/BounceEvolution/%s/%.*s.pvr", dirs[i], stem, base); if (path_exists(out)) return 1; }
        snprintf(out, sz, "ux0:data/BounceEvolution/%s/%s", dirs[i], base);
        if (path_exists(out) && strcmp(out + 15, vpath + 15) != 0) return 1;
    }
    return 0;
}
// Sound/music files are read completely into RAM once and then served from memory.
// The game streams them from inside the audio callback; reading the SD card there is slow and unpredictable.
#define SNDC_MAX 40
static struct { char path[200]; uint8_t *buf; size_t sz; } g_sndc[SNDC_MAX];
static int g_sndc_n = 0;
static FILE *sound_cache_open(const char *p) {
    size_t L = strlen(p);
    if (L < 5 || !(strstr(p, "/sounds/") || strstr(p, "/musics/"))) return NULL;
    const char *e = p + L - 4;
    if (strcasecmp(e, ".wav") && strcasecmp(e, ".mp3") && strcasecmp(e, ".ogg")) return NULL;
    for (int i = 0; i < g_sndc_n; i++)
        if (!strcmp(g_sndc[i].path, p)) return fmemopen(g_sndc[i].buf, g_sndc[i].sz, "rb");
    if (g_sndc_n >= SNDC_MAX || L >= sizeof(g_sndc[0].path)) return NULL;
    FILE *f = fopen(p, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *b = sz > 0 ? (uint8_t *)malloc((size_t)sz) : NULL;
    if (!b || fread(b, 1, (size_t)sz, f) != (size_t)sz) { free(b); fclose(f); return NULL; }
    fclose(f);
    strcpy(g_sndc[g_sndc_n].path, p); g_sndc[g_sndc_n].buf = b; g_sndc[g_sndc_n].sz = (size_t)sz; g_sndc_n++;
    vlog("[SNDC] cached %s (%ld bytes), %d files in RAM", p, sz, g_sndc_n);
    return fmemopen(b, (size_t)sz, "rb");
}
FILE *__wrap_fopen(const char *path, const char *mode) {
    char vpath[512];
    translate_path(path, vpath, sizeof(vpath));
    FILE *fp = NULL;
    if (mode && mode[0] == 'r' && !strchr(mode, '+')) fp = sound_cache_open(vpath);
    if (!fp) fp = fopen(vpath, mode);
    if (fp && mode && mode[0] == 'r' && !strchr(mode, '+')) setvbuf(fp, NULL, _IOFBF, 65536);   // default 1 KB buffer = hundreds of tiny SD reads per sound/texture
    if (!fp && mode && mode[0] == 'r') {
        char alt[512];
        if (asset_fallback(vpath, alt, sizeof(alt))) {
            fp = fopen(alt, mode);
            if (fp && !strchr(mode, '+')) setvbuf(fp, NULL, _IOFBF, 65536);
            vlog("fopen fallback '%s' -> '%s' = %p", vpath, alt, fp);
            return fp;
        }
    }
    if (g_verbose || !fp) vita_log("fopen('%s' -> '%s', '%s') = %p", path ? path : "NULL", vpath, mode, fp);
    return fp;
}
FILE *__wrap_freopen(const char *path, const char *mode, FILE *stream) {
    char vpath[512];
    translate_path(path, vpath, sizeof(vpath));
    return freopen(vpath, mode, stream);
}
int __wrap_open(const char *path, int flags, ...) {
    char vpath[512];
    translate_path(path, vpath, sizeof(vpath));
    // glibc (ARM Linux) open flags differ from Vita newlib's
    int nf = flags & 3;                 // O_RDONLY / O_WRONLY / O_RDWR
    if (flags & 0x40)  nf |= O_CREAT;
    if (flags & 0x80)  nf |= O_EXCL;
    if (flags & 0x200) nf |= O_TRUNC;
    if (flags & 0x400) nf |= O_APPEND;
    return open(vpath, nf, 0666);
}
int __wrap_remove(const char *path) {
    char vpath[512];
    translate_path(path, vpath, sizeof(vpath));
    return remove(vpath);
}
int __wrap_rename(const char *oldp, const char *newp) {
    char vo[512], vn[512];
    translate_path(oldp, vo, sizeof(vo));
    translate_path(newp, vn, sizeof(vn));
    return rename(vo, vn);
}

typedef struct glibc_stat {
    uint64_t st_dev;
    uint16_t __pad1;
    uint32_t st_ino;
    uint32_t st_mode;
    uint32_t st_nlink;
    uint32_t st_uid;
    uint32_t st_gid;
    uint64_t st_rdev;
    uint16_t __pad2;
    int32_t  st_size;
    int32_t  st_blksize;
    int32_t  st_blocks;
    int32_t  st_atime_sec;
    uint32_t st_atime_nsec;
    int32_t  st_mtime_sec;
    uint32_t st_mtime_nsec;
    int32_t  st_ctime_sec;
    uint32_t st_ctime_nsec;
    uint32_t __unused4;
    uint32_t __unused5;
} glibc_stat;

int __wrap___xstat(int ver, const char *path, glibc_stat *buf) {
    char vpath[512];
    translate_path(path, vpath, sizeof(vpath));
    struct stat st;
    if (stat(vpath, &st) != 0) return -1;
    if (buf) {
        memset(buf, 0, sizeof(*buf));
        buf->st_mode = st.st_mode;
        buf->st_size = (int32_t)st.st_size;
        buf->st_mtime_sec = (int32_t)st.st_mtime;
    }
    return 0;
}

typedef struct glibc_glob_t {
    size_t gl_pathc;
    char **gl_pathv;
    size_t gl_offs;
    int gl_flags;
    void *gl_closedir;
    void *gl_readdir;
    void *gl_opendir;
    void *gl_lstat;
    void *gl_stat;
} glibc_glob_t;

static int match_wildcard(const char *pat, const char *str) {
    if (*pat == '\0') return *str == '\0';
    if (*pat == '*') return match_wildcard(pat + 1, str) || (*str && match_wildcard(pat, str + 1));
    if (*pat == '?' || *pat == *str) return *str && match_wildcard(pat + 1, str + 1);
    return 0;
}

int __wrap_glob(const char *pattern, int flags, int (*errfunc)(const char *, int), glibc_glob_t *pglob) {
    if (!pattern || !pglob) return -1;
    char vpat[512];
    translate_path(pattern, vpat, sizeof(vpat));

    char dir_part[512];
    const char *slash = strrchr(vpat, '/');
    const char *file_pat = "*";
    if (slash) {
        size_t dlen = slash - vpat;
        memcpy(dir_part, vpat, dlen);
        dir_part[dlen] = '\0';
        file_pat = slash + 1;
    } else {
        strcpy(dir_part, "ux0:data/BounceEvolution");
        file_pat = vpat;
    }

    char orig_dir[512] = "";
    const char *orig_slash = strrchr(pattern, '/');
    if (orig_slash) {
        size_t olen = orig_slash - pattern;
        memcpy(orig_dir, pattern, olen);
        orig_dir[olen] = '\0';
    }

    SceUID dfd = sceIoDopen(dir_part);
    if (dfd < 0) return 3; // GLOB_NOMATCH

    char **list = (char **)calloc(256, sizeof(char *));
    size_t count = 0;
    SceIoDirent dent;
    while (sceIoDread(dfd, &dent) > 0 && count < 255) {
        if (strcmp(dent.d_name, ".") == 0 || strcmp(dent.d_name, "..") == 0) continue;
        if (match_wildcard(file_pat, dent.d_name)) {
            char full[512];
            if (orig_slash) snprintf(full, sizeof(full), "%s/%s", orig_dir, dent.d_name);
            else snprintf(full, sizeof(full), "%s", dent.d_name);
            list[count++] = strdup(full);
        }
    }
    sceIoDclose(dfd);

    if (count == 0) {
        free(list);
        return 3; // GLOB_NOMATCH
    }
    pglob->gl_pathc = count;
    pglob->gl_pathv = list;
    pglob->gl_offs = 0;
    return 0;
}

void __wrap_globfree(glibc_glob_t *pglob) {
    if (!pglob || !pglob->gl_pathv) return;
    for (size_t i = 0; i < pglob->gl_pathc; i++) {
        free(pglob->gl_pathv[i]);
    }
    free(pglob->gl_pathv);
    pglob->gl_pathv = NULL;
    pglob->gl_pathc = 0;
}

// Standard C library & POSIX passthroughs
int __wrap__IO_getc(FILE *fp) { return getc(fp); }
int *__wrap___errno_location(void) { return __errno(); }
void __wrap_abort(void) { vita_log("abort() called"); sceKernelExitProcess(1); while (1) {} }
void __wrap_exit(int status) { vita_log("exit(%d) called", status); sceKernelExitProcess(status); while (1) {} }
void __wrap_clearerr(FILE *fp) { clearerr(fp); }
clock_t __wrap_clock(void) { return clock(); }
int __wrap_close(int fd) { return close(fd); }
int __wrap_fclose(FILE *fp) { return fclose(fp); }
int __wrap_fcntl(int fd, int cmd, ...) { return 0; }
int __wrap_feof(FILE *fp) { return feof(fp); }
int __wrap_ferror(FILE *fp) { return ferror(fp); }
int __wrap_fflush(FILE *fp) { return fflush(fp); }
char *__wrap_fgets(char *s, int size, FILE *fp) { return fgets(s, size, fp); }
int __wrap_fprintf(FILE *fp, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); int r = vfprintf(fp, fmt, ap); va_end(ap); return r;
}
int __wrap_printf(const char *fmt, ...) {
    char buf[512];
    va_list ap; va_start(ap, fmt); int r = vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    vita_log("[GAME] %s", buf);
    return r;
}
int __wrap_sprintf(char *str, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); int r = vsprintf(str, fmt, ap); va_end(ap); return r;
}
int __wrap_fscanf(FILE *fp, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); int r = vfscanf(fp, fmt, ap); va_end(ap); return r;
}
int __wrap_sscanf(const char *str, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); int r = vsscanf(str, fmt, ap); va_end(ap); return r;
}
int __wrap_fputc(int c, FILE *fp) { return fputc(c, fp); }
int __wrap_fputs(const char *s, FILE *fp) { return fputs(s, fp); }
size_t __wrap_fread(void *ptr, size_t size, size_t nmemb, FILE *fp) { return fread(ptr, size, nmemb, fp); }
size_t __wrap_fwrite(const void *ptr, size_t size, size_t nmemb, FILE *fp) { return fwrite(ptr, size, nmemb, fp); }
int __wrap_fseek(FILE *fp, long offset, int whence) { return fseek(fp, offset, whence); }
long __wrap_ftell(FILE *fp) { return ftell(fp); }
int __wrap_ungetc(int c, FILE *fp) { return ungetc(c, fp); }
int __wrap_setvbuf(FILE *fp, char *buf, int mode, size_t size) { return setvbuf(fp, buf, mode, size); }
FILE *__wrap_tmpfile(void) { return fopen("ux0:data/BounceEvolution/tmpfile.bin", "wb+"); }
char *__wrap_tmpnam(char *s) {
    static char buf[64] = "ux0:data/BounceEvolution/tmp_0";
    if (s) { strcpy(s, buf); return s; }
    return buf;
}
void *__wrap_malloc(size_t sz) { return malloc(sz); }
void __wrap_free(void *p) { free(p); }
void *__wrap_realloc(void *p, size_t sz) { return realloc(p, sz); }
char *__wrap_getenv(const char *name) { return getenv(name); }
// Game clock is anchored to "process start" so the first delta the engine sees is small
static uint64_t g_start_time_us = 0;
// Slow frames (and level-load stalls) are not passed on to the game as one huge time step:
// every frame longer than FRAME_CAP_US adds its excess to g_time_debt_us, which the game clock subtracts.
// The game then slows down a little in heavy scenes instead of taking big physics steps ("ball too fast").
static uint64_t get_game_time_usec(void) {
    uint64_t n = sceKernelGetProcessTimeWide();
    if (g_start_time_us == 0) g_start_time_us = n;
    uint64_t t = n - g_start_time_us + 1000;
    uint64_t d = g_time_debt_us;
    t = (t > d) ? t - d : 1000;
    if (t < g_last_game_us) t = g_last_game_us;      // never go backwards
    g_last_game_us = t;
    return t;
}
int __wrap_gettimeofday(struct timeval *tv, void *tz) {
    uint64_t t = get_game_time_usec();
    if (tv) { tv->tv_sec = t / 1000000; tv->tv_usec = t % 1000000; }
    return 0;
}
struct tm *__wrap_gmtime(const time_t *t) { return gmtime(t); }
struct tm *__wrap_localtime(const time_t *t) { return localtime(t); }
time_t __wrap_mktime(struct tm *tm) { return mktime(tm); }
time_t __wrap_time(time_t *t) { return time(t); }
size_t __wrap_strftime(char *s, size_t max, const char *fmt, const struct tm *tm) { return strftime(s, max, fmt, tm); }
struct lconv *__wrap_localeconv(void) { return localeconv(); }
char *__wrap_setlocale(int category, const char *locale) { return setlocale(category, locale); }
int __wrap_isalnum(int c) { return isalnum(c); }
int __wrap_isalpha(int c) { return isalpha(c); }
int __wrap_iscntrl(int c) { return iscntrl(c); }
int __wrap_islower(int c) { return islower(c); }
int __wrap_ispunct(int c) { return ispunct(c); }
int __wrap_isspace(int c) { return isspace(c); }
int __wrap_isupper(int c) { return isupper(c); }
int __wrap_isxdigit(int c) { return isxdigit(c); }
int __wrap_tolower(int c) { return tolower(c); }
int __wrap_toupper(int c) { return toupper(c); }
off_t __wrap_lseek(int fd, off_t offset, int whence) { return lseek(fd, offset, whence); }
ssize_t __wrap_read(int fd, void *buf, size_t count) { return read(fd, buf, count); }
void *__wrap_memchr(const void *s, int c, size_t n) { return memchr(s, c, n); }
int __wrap_memcmp(const void *s1, const void *s2, size_t n) { return memcmp(s1, s2, n); }
void *__wrap_memcpy(void *dest, const void *src, size_t n) { return memcpy(dest, src, n); }
void *__wrap_memmove(void *dest, const void *src, size_t n) { return memmove(dest, src, n); }
void *__wrap_memset(void *s, int c, size_t n) { return memset(s, c, n); }
int __wrap_strcasecmp(const char *s1, const char *s2) { return strcasecmp(s1, s2); }
int __wrap_strncasecmp(const char *s1, const char *s2, size_t n) { return strncasecmp(s1, s2, n); }
char *__wrap_strcat(char *dest, const char *src) { return strcat(dest, src); }
char *__wrap_strncat(char *dest, const char *src, size_t n) { return strncat(dest, src, n); }
char *__wrap_strchr(const char *s, int c) { return strchr(s, c); }
char *__wrap_strrchr(const char *s, int c) { return strrchr(s, c); }
int __wrap_strcmp(const char *s1, const char *s2) { return strcmp(s1, s2); }
int __wrap_strncmp(const char *s1, const char *s2, size_t n) { return strncmp(s1, s2, n); }
int __wrap_strcoll(const char *s1, const char *s2) { return strcoll(s1, s2); }
char *__wrap_strcpy(char *dest, const char *src) { return strcpy(dest, src); }
char *__wrap_strncpy(char *dest, const char *src, size_t n) { return strncpy(dest, src, n); }
size_t __wrap_strcspn(const char *s, const char *reject) { return strcspn(s, reject); }
char *__wrap_strerror(int errnum) { return strerror(errnum); }
size_t __wrap_strlen(const char *s) { return strlen(s); }
char *__wrap_strpbrk(const char *s, const char *accept) { return strpbrk(s, accept); }
char *__wrap_strstr(const char *haystack, const char *needle) { return strstr(haystack, needle); }
char *__wrap_strtok(char *str, const char *delim) { return strtok(str, delim); }
int __wrap_rand(void) { return rand(); }
void __wrap_srand(unsigned int seed) { srand(seed); }
int __wrap_sched_yield(void) { sceKernelDelayThread(100); return 0; }
int __wrap_usleep(useconds_t usec) {
    if (usec > 16000) usec = 16000;                    // never sleep longer than one frame
    return sceKernelDelayThread(usec);
}
int __wrap_select(int nfds, fd_set *r, fd_set *w, fd_set *e, struct timeval *timeout) {
    uint64_t us = 16000;
    if (timeout) {
        us = (uint64_t)timeout->tv_sec * 1000000ull + (uint64_t)timeout->tv_usec;
        if (us > 16000) us = 16000;
    }
    if (us) sceKernelDelayThread((SceUInt32)us);
    if (r) FD_ZERO(r);
    if (w) FD_ZERO(w);
    if (e) FD_ZERO(e);
    return 0;
}
int __wrap_system(const char *cmd) { return 0; }

// glibc 24-byte pthread_mutex_t -> Vita newlib pthread_mutex_t bridge
static pthread_mutex_t *get_vita_mutex(void **m) {
    if (!*m) {
        pthread_mutex_t *vm = (pthread_mutex_t *)calloc(1, sizeof(pthread_mutex_t));
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
        pthread_mutex_init(vm, &attr);
        pthread_mutexattr_destroy(&attr);
        *m = vm;
    }
    return (pthread_mutex_t *)(*m);
}
int __wrap_pthread_mutex_init(void **m, const void *attr) {
    *m = NULL;
    get_vita_mutex(m);
    return 0;
}
int __wrap_pthread_mutex_destroy(void **m) {
    if (m && *m) {
        pthread_mutex_destroy((pthread_mutex_t *)(*m));
        free(*m);
        *m = NULL;
    }
    return 0;
}
int __wrap_pthread_mutex_lock(void **m) { return pthread_mutex_lock(get_vita_mutex(m)); }
int __wrap_pthread_mutex_trylock(void **m) { return pthread_mutex_trylock(get_vita_mutex(m)); }
int __wrap_pthread_mutex_unlock(void **m) { return pthread_mutex_unlock(get_vita_mutex(m)); }
int __wrap_pthread_mutexattr_init(void *attr) { return 0; }
int __wrap_pthread_mutexattr_destroy(void *attr) { return 0; }
int __wrap_pthread_mutexattr_settype(void *attr, int type) { return 0; }

int __wrap_pthread_create(pthread_t *thread, const void *attr, void *(*start_routine)(void *), void *arg) {
    return pthread_create(thread, NULL, start_routine, arg);
}
void __wrap_pthread_exit(void *retval) { pthread_exit(retval); }
int __wrap_pthread_join(pthread_t thread, void **retval) { return pthread_join(thread, retval); }

// C++ ABI / AEABI Passthroughs
extern void *_Znwj(size_t);
extern void *_Znaj(size_t);
extern void _ZdlPv(void *);
extern void _ZdaPv(void *);
extern void _ZSt9terminatev(void);
extern void *__cxa_allocate_exception(size_t);
extern void *__cxa_begin_catch(void *);
extern void __cxa_end_catch(void);
extern void __cxa_end_cleanup(void);
extern void *__cxa_get_exception_ptr(void *);
extern void __cxa_guard_abort(void *);
extern int  __cxa_guard_acquire(void *);
extern void __cxa_guard_release(void *);
extern void __cxa_pure_virtual(void);
extern void __cxa_throw(void *, void *, void (*)(void *));
extern void *__dynamic_cast(const void *, const void *, const void *, ptrdiff_t);
extern int  __gxx_personality_v0(int, int, uint64_t, void *, void *);
extern int  __aeabi_idiv(int, int);
extern int  __aeabi_idivmod(int, int);
extern unsigned int __aeabi_uidiv(unsigned int, unsigned int);
extern unsigned int __aeabi_uidivmod(unsigned int, unsigned int);
extern const void *_ZTISt9bad_alloc[];

// ---------------------------------------------------------------------------
// C++ ABI entry points are pure tail-jump thunks (no stack frame, no C wrapper).
// This matters: a C++ exception thrown by the game is unwound frame by frame,
// and a normal C wrapper in between has no .ARM.exidx entry, so the unwinder
// would give up ("end of stack") and call std::terminate -> abort.
// ---------------------------------------------------------------------------
#define TAILJMP(wrap, target) \
    __attribute__((naked, used)) void wrap(void) { __asm__ volatile("b " #target "\n"); }
TAILJMP(__wrap__Znwj, _Znwj)
TAILJMP(__wrap__Znaj, _Znaj)
TAILJMP(__wrap__ZdlPv, _ZdlPv)
TAILJMP(__wrap__ZdaPv, _ZdaPv)
TAILJMP(__wrap__ZSt9terminatev, _ZSt9terminatev)
TAILJMP(__wrap___cxa_allocate_exception, __cxa_allocate_exception)
TAILJMP(__wrap___cxa_begin_catch, __cxa_begin_catch)
TAILJMP(__wrap___cxa_end_catch, __cxa_end_catch)
TAILJMP(__wrap___cxa_end_cleanup, __cxa_end_cleanup)
TAILJMP(__wrap___cxa_get_exception_ptr, __cxa_get_exception_ptr)
TAILJMP(__wrap___cxa_guard_abort, __cxa_guard_abort)
TAILJMP(__wrap___cxa_guard_acquire, __cxa_guard_acquire)
TAILJMP(__wrap___cxa_guard_release, __cxa_guard_release)
TAILJMP(__wrap___cxa_pure_virtual, __cxa_pure_virtual)
TAILJMP(__wrap___cxa_throw, __cxa_throw)
TAILJMP(__wrap___dynamic_cast, __dynamic_cast)
TAILJMP(__wrap___gxx_personality_v0, __gxx_personality_v0)
int   __wrap___aeabi_atexit(void *obj, void (*dtor)(void *), void *dso) { return 0; }
int   __wrap___aeabi_idiv(int a, int b) { return __aeabi_idiv(a, b); }
int   __wrap___aeabi_idivmod(int a, int b) { return __aeabi_idivmod(a, b); }
unsigned int __wrap___aeabi_uidiv(unsigned int a, unsigned int b) { return __aeabi_uidiv(a, b); }
unsigned int __wrap___aeabi_uidivmod(unsigned int a, unsigned int b) { return __aeabi_uidivmod(a, b); }
void *__wrap__ZTISt9bad_alloc = (void *)_ZTISt9bad_alloc;
void *__wrap___gmon_start__ = NULL;
int   __wrap___libc_start_main(int (*main_fn)(int, char **, char **), int argc, char **ubp_av,
                               void (*init)(void), void (*fini)(void), void (*rtld_fini)(void), void *stack_end) {
    return main_fn(argc, ubp_av, NULL);
}


// ============================================================================
// ARM EABI C++ Exception Unwind Table Bridge (.ARM.exidx)
// ============================================================================
extern uint32_t __exidx_start[];
extern uint32_t __exidx_end[];

void *__gnu_Unwind_Find_exidx(void *pc, int *pcount) {
    uintptr_t addr = (uintptr_t)pc;
    uintptr_t rx_start = (uintptr_t)bounce_rx_base;
    uintptr_t rx_end   = rx_start + (0x001c6520 - 0x00008000);
    if (addr >= rx_start && addr < rx_end) {
        // bounce's .ARM.exidx is at vaddr 0x001bf27c, size 0x72a0 (3668 entries of 8 bytes)
        *pcount = 0x72a0 / 8;
        return (void *)(bounce_rx_base + (0x001bf27c - 0x00008000));
    }
    *pcount = (__exidx_end - __exidx_start) / 2;
    return (void *)__exidx_start;
}

// ============================================================================
// 9. PS Vita Entry Point
// ============================================================================
extern const void *_ZTVN10__cxxabiv117__class_type_infoE[];
extern const void *_ZTVN10__cxxabiv119__pointer_type_infoE[];
extern const void *_ZTVN10__cxxabiv120__si_class_type_infoE[];
extern const void *_ZTVN10__cxxabiv121__vmi_class_type_infoE[];

int main(int argc, char *argv[]) {
    {   // one-time migration from the old folder name (keeps game data, vita.cfg and logs)
        SceIoStat st;
        if (sceIoGetstat("ux0:data/BounceEvolution", &st) < 0 && sceIoGetstat("ux0:data/bounce", &st) >= 0)
            sceIoRename("ux0:data/bounce", "ux0:data/BounceEvolution");
    }
    sceIoMkdir("ux0:data/BounceEvolution", 0777);
    FILE *cf = fopen("ux0:data/BounceEvolution/bounce.conf", "ab"); if (cf) fclose(cf);
    remove("ux0:data/BounceEvolution/vita_log.txt");
    { SceIoStat st; g_verbose = (sceIoGetstat("ux0:data/BounceEvolution/debug.txt", &st) >= 0); }
    pa_mutex_make_recursive();
    load_cfg();
    {
        scePowerSetArmClockFrequency(444);
        scePowerSetBusClockFrequency(222);
        scePowerSetGpuClockFrequency(222);
        scePowerSetGpuXbarClockFrequency(166);
    }
    vita_log("=== Bounce Evolution Vita Port v5.1 Starting (verbose=%d) ===", g_verbose);
#ifdef __OPTIMIZE__
    vita_log("[BUILD] optimized build");
#else
    vita_log("[BUILD] NOT optimized (-O0): add -O2 to CMAKE_C_FLAGS");
#endif
    vita_log("bounce_rx_base = %p, bounce_rw_base = %p", bounce_rx_base, bounce_rw_base);

    // Initialize sensors & controls
    sceCtrlSetSamplingMode(SCE_CTRL_MODE_ANALOG);
    sceTouchSetSamplingState(SCE_TOUCH_PORT_FRONT, SCE_TOUCH_SAMPLING_STATE_START);
    sceMotionReset();
    sceMotionStartSampling();
    vita_log("Sensors initialized.");

    // Initialize vitaGL with runtime shader compiler (vitaShaRK) enabled
    vglSetShaderCachePath("ux0:data/shader_cache/BNCE00001");   // no-op unless vitaGL was built with HAVE_SHADER_CACHE=1
    vglSetupRuntimeShaderCompiler(SHARK_OPT_UNSAFE, SHARK_ENABLE, SHARK_ENABLE, SHARK_ENABLE);
    vglInitExtended(0, 960, 544, 8 * 1024 * 1024,
                    g_msaa == 4 ? SCE_GXM_MULTISAMPLE_4X : (g_msaa == 2 ? SCE_GXM_MULTISAMPLE_2X : SCE_GXM_MULTISAMPLE_NONE));
    vita_log("[GFX] msaa=%d", g_msaa);
    vita_log("vitaGL initialized.");

    // Populate R_ARM_COPY globals in bounce's .bss (0x001d12d8..0x001d1370)
    *(FILE **)(bounce_rw_base + (0x001d12d8 - 0x001cf000)) = stdin;
    memcpy(bounce_rw_base + (0x001d12e0 - 0x001cf000), _ZTVN10__cxxabiv117__class_type_infoE, 48);
    *(FILE **)(bounce_rw_base + (0x001d1310 - 0x001cf000)) = stderr;
    memcpy(bounce_rw_base + (0x001d1318 - 0x001cf000), _ZTVN10__cxxabiv119__pointer_type_infoE, 40);
    memcpy(bounce_rw_base + (0x001d1340 - 0x001cf000), _ZTVN10__cxxabiv120__si_class_type_infoE, 44);
    *(FILE **)(bounce_rw_base + (0x001d136c - 0x001cf000)) = stdout;
    memcpy(bounce_rw_base + (0x001d1370 - 0x001cf000), _ZTVN10__cxxabiv121__vmi_class_type_infoE, 48);
    vita_log("R_ARM_COPY globals populated.");

    // Run bounce's static C++ constructors (.init_array at 0x001cf000, 9 entries = 36 bytes)
    typedef void (*init_fn_t)(int, char **, char **);
    init_fn_t *init_arr = (init_fn_t *)(bounce_rw_base + 0);
    char *fake_argv[] = { (char *)"bounce", NULL };
    for (int i = 0; i < 9; i++) {
        vlog("Running init_array[%d] @ %p...", i, init_arr[i]);
        if (init_arr[i]) init_arr[i](1, fake_argv, NULL);
    }

    vita_log("Entering bounce_main...");
    int ret = bounce_main(1, fake_argv);
    vita_log("bounce_main returned %d", ret);
    return ret;
}





// ---------------------------------------------------------------------------
// Workaround: the vitaGL GLSL translator / SceShaccCg fail to compile the array
// varying  "varying<TAB>mediump<TAB>vec2<TAB>V_BlurTexCoord[5];"  used by the
// BLUR_1D post-processing shaders (the failed compile leaves a NULL program and
// glLinkProgram then crashes inside SceGxm).  We rewrite the source so the array
// becomes five plain vec2 varyings V_BlurTexCoord0..4.  Whitespace-tolerant.
// ---------------------------------------------------------------------------
#define SPLIT_VARYING_ARRAYS 1
static int is_ident_char(int c) { return isalnum(c) || c == '_'; }

// Matches  V_BlurTexCoord <ws> [ <ws> digits <ws> ]  at p. Returns match length (0 = no), index in *idx.
static size_t match_blur_index(const char *p, int *idx) {
    static const char name[] = "V_BlurTexCoord";
    size_t nl = sizeof(name) - 1;
    if (strncmp(p, name, nl) != 0 || is_ident_char((unsigned char)p[nl])) return 0;
    const char *q = p + nl;
    while (*q == ' ' || *q == '\t') q++;
    if (*q != '[') return 0;
    q++;
    while (*q == ' ' || *q == '\t') q++;
    if (!isdigit((unsigned char)*q)) return 0;
    int v = 0;
    while (isdigit((unsigned char)*q)) { v = v * 10 + (*q - '0'); q++; }
    while (*q == ' ' || *q == '\t') q++;
    if (*q != ']') return 0;
    q++;
    *idx = v;
    return (size_t)(q - p);
}

static char *split_blur_varyings(const char *src) {
    size_t n = strlen(src);
    char *out = (char *)malloc(n + 4096);
    if (!out) return NULL;
    size_t o = 0;
    for (size_t i = 0; i < n; ) {
        // declaration:  varying ... V_BlurTexCoord[N] ... ;
        if ((i == 0 || !is_ident_char((unsigned char)src[i - 1])) && strncmp(src + i, "varying", 7) == 0 && !is_ident_char((unsigned char)src[i + 7])) {
            size_t j = i + 7;
            size_t lim = i + 160 < n ? i + 160 : n;
            while (j < lim && src[j] != ';' && src[j] != '\n' && src[j] != '{') j++;
            int idx = -1; size_t found = 0;
            if (j < lim && src[j] == ';') {
                for (size_t k = i + 7; k < j; k++) {
                    int v;
                    size_t m = match_blur_index(src + k, &v);
                    if (m) { idx = v; found = m; break; }
                }
            }
            if (found && idx > 0 && idx <= 8) {
                for (int c = 0; c < idx; c++) o += (size_t)sprintf(out + o, "varying mediump vec2 V_BlurTexCoord%d;\n", c);
                i = j + 1;
                continue;
            }
        }
        // usage: V_BlurTexCoord[k] -> V_BlurTexCoordk
        if (src[i] == 'V' && (i == 0 || !is_ident_char((unsigned char)src[i - 1]))) {
            int v;
            size_t m = match_blur_index(src + i, &v);
            if (m) { o += (size_t)sprintf(out + o, "V_BlurTexCoord%d", v); i += m; continue; }
        }
        out[o++] = src[i++];
    }
    out[o] = 0;
    return out;
}

void __wrap_glShaderSource(GLuint sh, GLsizei count, const GLchar *const *string, const GLint *length) {
    if (g_verbose && count > 0 && string && string[0]) {
        int ln = (length && length[0] >= 0 && length[0] < 240) ? length[0] : 240;
        vita_log("[SHADER SRC] sh=%u, count=%d, length=%s:\n%.*s", (unsigned)sh, (int)count, length ? "set" : "NULL", ln, string[0]);
    }
#if SPLIT_VARYING_ARRAYS
    if (count > 0 && string) {
        // Join all pieces (honouring `length`) into one NUL-terminated buffer
        size_t total = 0;
        for (GLsizei i = 0; i < count; i++) {
            if (!string[i]) continue;
            total += (length && length[i] >= 0) ? (size_t)length[i] : strlen(string[i]);
        }
        char *all = (char *)malloc(total + 1);
        if (all) {
            size_t pos = 0;
            for (GLsizei i = 0; i < count; i++) {
                if (!string[i]) continue;
                size_t len = (length && length[i] >= 0) ? (size_t)length[i] : strlen(string[i]);
                memcpy(all + pos, string[i], len);
                pos += len;
            }
            all[pos] = 0;
            if (strstr(all, "V_BlurTexCoord")) {
                char *patched = split_blur_varyings(all);
                free(all);
                if (patched) {
                    const GLchar *arr[1] = { patched };
                    vlog("[SHADER PATCH] sh=%u: split V_BlurTexCoord[] (count=%d, length=%s)",
                             (unsigned)sh, (int)count, length ? "set" : "NULL");
                    glShaderSource(sh, 1, arr, NULL);
                    return;   // not freed on purpose, in case vitaGL keeps the pointer
                }
            } else {
                free(all);
            }
        }
    }
#endif
    glShaderSource(sh, count, string, length);
}

void __wrap_glCompileShader(GLuint sh) {
    glCompileShader(sh);                      // with the POSTPONED GLSL mode this is a no-op; real compile happens in glLinkProgram
    if (g_verbose) {
        GLint status = 0;
        glGetShaderiv(sh, GL_COMPILE_STATUS, &status);
        vita_log("[SHADER COMPILE] sh=%u, status=%d", (unsigned)sh, (int)status);
        if (!status) {
            GLchar log[512];
            GLsizei len = 0;
            glGetShaderInfoLog(sh, sizeof(log), &len, log);
            vita_log("[SHADER COMPILE ERROR] sh=%u: %s", (unsigned)sh, log);
        }
    }
}

static uint32_t g_link_total_ms = 0, g_link_count = 0;
void __wrap_glLinkProgram(GLuint prog) {
    vlog("[PROGRAM LINK BEGIN] prog=%u", (unsigned)prog);
    // Written before the call so that a crash inside vitaGL/SceGxm still leaves a marker in the log
    if (!g_verbose) vlog("[LINK] prog=%u ...", (unsigned)prog);
    uint32_t t0 = sceKernelGetProcessTimeLow();
    glLinkProgram(prog);
    uint32_t ms = (sceKernelGetProcessTimeLow() - t0) / 1000;
    g_link_total_ms += ms; g_link_count++;
    GLint status = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &status);
    vlog("[LINK] prog=%u done in %u ms, status=%d (total %u programs, %u ms)", (unsigned)prog, (unsigned)ms, (int)status, (unsigned)g_link_count, (unsigned)g_link_total_ms);
    if (!status) {
        GLchar log[512];
        GLsizei len = 0;
        glGetProgramInfoLog(prog, sizeof(log), &len, log);
        vita_log("[PROGRAM LINK ERROR] prog=%u: %s", (unsigned)prog, log);
    }
}
