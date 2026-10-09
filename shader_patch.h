// shader_patch.h - fixes the stock N900 shader sources at glShaderSource time so the
// original, unmodified game data works on the Vita's shader compiler (SceShaccCg).
// Pure C, no Vita dependencies (unit-testable on the host).
//
// By the time glShaderSource is called the game has already flattened its own #include
// directives, so the source contains the stock commons/varyings.sh block verbatim:
//     varying mediump vec2 V_BlurTexCoord[5]; ... (11 unguarded varyings)
// We rewrite it as follows:
//   * CRLF -> LF
//   * the unguarded varyings run is replaced by  defines.sh + guarded varyings, with
//     V_BlurTexCoord[5] split into V_BlurTexCoord0..4. Each varying sits in its
//     #ifdef NEEDS_Vxxx guard (macros come from defines.sh) so unused varyings are not
//     declared, keeping the interpolant count within what the Vita supports.
//   * V_BlurTexCoord[i] -> V_BlurTexCoordi
//   * diff-tex.vs: 'worldtangent' is declared before 'worldbinormal' uses it
#ifndef SHADER_PATCH_H
#define SHADER_PATCH_H
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

static const char SP_VARYINGS[] =
"#ifdef NEEDS_VTEXCOORD0\nvarying mediump vec2 V_TexCoord0;\n#endif\n"
"#ifdef NEEDS_VTEXCOORD1\nvarying mediump vec2 V_TexCoord1;\n#endif\n"
"#ifdef NEEDS_VBLURTEXCOORD\n"
"varying mediump vec2 V_BlurTexCoord0;\nvarying mediump vec2 V_BlurTexCoord1;\n"
"varying mediump vec2 V_BlurTexCoord2;\nvarying mediump vec2 V_BlurTexCoord3;\n"
"varying mediump vec2 V_BlurTexCoord4;\n#endif\n"
"varying highp float V_LinearDepth;\n"
"#ifdef NEEDS_VFOGLEVEL\nvarying lowp float V_FogLevel;\n#endif\n"
"#ifdef NEEDS_VTOLIGHT\nvarying mediump vec3 V_ToLight;\n#endif\n"
"#ifdef NEEDS_VNORMAL\nvarying mediump vec3 V_Normal;\n#endif\n"
"#ifdef NEEDS_VTOCAMERA\nvarying lowp vec3 V_ToCamera;\n#endif\n"
"#ifdef NEEDS_VPOSITION\nvarying highp vec3 V_Position;\n#endif\n"
"#ifdef NEEDS_VTANGENTTM\nvarying lowp mat3 V_TangentTM;\n#endif\n"
"#ifdef NEEDS_VVERTEXCOLOR\nvarying lowp vec4 V_VertexColor;\n#endif\n";

static char *sp_replace_one(char *s, size_t *n, const char *from, const char *to) {
    char *p = strstr(s, from);
    if (!p) return s;
    size_t fl = strlen(from), tl = strlen(to);
    char *o = (char *)malloc(*n - fl + tl + 1);
    if (!o) return s;
    size_t pre = (size_t)(p - s);
    memcpy(o, s, pre); memcpy(o + pre, to, tl); memcpy(o + pre + tl, p + fl, *n - pre - fl + 1);
    free(s);
    *n = *n - fl + tl;
    return o;
}

// Returns a malloc'd patched copy, or NULL if nothing needed patching / on failure.
// `defines` is the (CRLF-stripped or not) text of commons/defines.sh, or NULL.
static char *sp_patch_source(const char *src, size_t n, const char *defines, size_t *outn) {
    int changed = 0;
    char *a = (char *)malloc(n + 1);
    if (!a) return NULL;
    size_t m = 0;
    for (size_t i = 0; i < n; i++) {
        if (src[i] == '\r') { changed = 1; continue; }
        a[m++] = src[i];
    }
    a[m] = 0;

    // the stock varyings run: consecutive lines starting with "varying", one of them V_BlurTexCoord[
    char *bt = strstr(a, "V_BlurTexCoord[");
    if (bt && defines) {
        char *ls = bt; while (ls > a && ls[-1] != '\n') ls--;
        char *first = ls;
        while (first > a) {                           // walk back over preceding "varying" lines
            char *pl = first - 1;
            if (pl < a) break;
            char *ps = pl; while (ps > a && ps[-1] != '\n') ps--;
            if (strncmp(ps, "varying", 7) != 0) break;
            first = ps;
        }
        char *last = ls;
        for (;;) {                                    // walk forward over "varying" lines
            char *e = strchr(last, '\n');
            if (!e) { last = a + m; break; }
            last = e + 1;
            if (strncmp(last, "varying", 7) != 0) break;
        }
        if (!strncmp(first, "varying", 7)) {
            size_t dl = strlen(defines), vl = sizeof(SP_VARYINGS) - 1;
            size_t pre = (size_t)(first - a), post = (size_t)((a + m) - last);
            char *o = (char *)malloc(pre + dl + 2 + vl + post + 1);
            if (o) {
                size_t k = 0;
                memcpy(o, a, pre); k = pre;
                // defines text with CR removed
                for (size_t i = 0; i < dl; i++) if (defines[i] != '\r') o[k++] = defines[i];
                if (k && o[k - 1] != '\n') o[k++] = '\n';
                memcpy(o + k, SP_VARYINGS, vl); k += vl;
                memcpy(o + k, last, post); k += post;
                o[k] = 0;
                free(a); a = o; m = k; changed = 1;
            }
        }
    }
    // remaining V_BlurTexCoord[d] uses
    {
        char *p = a;
        while ((p = strstr(p, "V_BlurTexCoord[")) != NULL) {
            if (isdigit((unsigned char)p[15]) && p[16] == ']') {
                p[14] = p[15];
                memmove(p + 15, p + 17, strlen(p + 17) + 1);
                m -= 2; changed = 1;
            } else p++;
        }
    }
    {
        size_t mm = m;
        a = sp_replace_one(a,&mm,
            "\tmediump vec3 worldbinormal = cross( normalize(V_Normal), worldtangent );\n\tmediump vec3 worldtangent = normalize(mat3(WORLDTM) * DT_TANGENT);\n",
            "\tmediump vec3 worldtangent = normalize(mat3(WORLDTM) * DT_TANGENT);\n\tmediump vec3 worldbinormal = cross( normalize(V_Normal), worldtangent );\n");
        if (mm != m) changed = 1;
        m = mm;
    }
    if (!changed) { free(a); return NULL; }
    *outn = m;
    return a;
}
#endif
