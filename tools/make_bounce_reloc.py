import os
import struct
import subprocess
import re

ELF_PATH = os.environ.get("BOUNCE_ELF", os.path.expanduser("~/bounce_vita/extract/opt/bounce/bin/bounce"))
RX_START = 0x00008000
RX_END   = 0x001c6520
RW_START = 0x001cf000
DATA_END = 0x001d12d8
RW_END   = 0x001d8e78
PMIN     = 0x0000c858

with open(ELF_PATH, "rb") as f:
    elf_data = f.read()

rx_img = bytearray(elf_data[0x000000:0x1be520])
rw_img = bytearray(elf_data[0x1bf000:0x1c12d8])

def get_u32(vaddr):
    if RX_START <= vaddr < RX_END:
        return struct.unpack_from("<I", rx_img, vaddr - RX_START)[0]
    elif RW_START <= vaddr < DATA_END:
        return struct.unpack_from("<I", rw_img, vaddr - RW_START)[0]
    return 0

def set_u32(vaddr, val):
    if RX_START <= vaddr < RX_END:
        struct.pack_into("<I", rx_img, vaddr - RX_START, val)
    elif RW_START <= vaddr < DATA_END:
        struct.pack_into("<I", rw_img, vaddr - RW_START, val)

# 1. Parse ELF sections
sec_out = subprocess.check_output(["arm-vita-eabi-readelf", "-S", "-W", ELF_PATH], text=True)
sections = {}
for line in sec_out.splitlines():
    m = re.match(r"\s*\[\s*(\d+)\]\s+(\S+)\s+(\S+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)", line)
    if m:
        idx, name, stype, addr, off, size = m.groups()
        sections[name] = (int(idx), stype, int(addr, 16), int(off, 16), int(size, 16))

# 2. Parse symbol table (.symtab)
sym_out = subprocess.check_output(["arm-vita-eabi-readelf", "-s", "-W", ELF_PATH], text=True)
sec12_map = []
sec14_syms = []
sec23_syms = []
func_addrs = set()

for line in sym_out.splitlines():
    m = re.match(r"\s*\d+:\s+([0-9a-fA-F]+)\s+(\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(.*)", line)
    if m:
        val, size, stype, bind, vis, ndx, name = m.groups()
        if not ndx.isdigit():
            continue
        val = int(val, 16)
        size = int(size)
        name = name.strip()
        if stype == "FUNC" and val >= PMIN:
            func_addrs.add(val)
        if ndx == "12" and name in ("$a", "$d", "$t"):
            sec12_map.append((val, name))
        elif ndx == "14" and name not in ("$a", "$d", "$t"):
            sec14_syms.append((val, size, stype, name))
        elif ndx == "23" and name not in ("$a", "$d", "$t"):
            sec23_syms.append((val, size, stype, name))

sec12_map.sort(key=lambda x: x[0])
sec14_syms.sort(key=lambda x: x[0])
sec23_syms.sort(key=lambda x: x[0])

# 3. Scan .text disassembly for VLDR float loads and switch jump tables (ldrls pc, [pc, rX, lsl #2])
dis_out = subprocess.check_output(["arm-vita-eabi-objdump", "-d", "-j", ".text", ELF_PATH], text=True)
vldr_targets = set()
jump_table_starts = set()

for line in dis_out.splitlines():
    m = re.search(r"^\s*([0-9a-fA-F]+):\s+[0-9a-fA-F]+\s+vldr\S*\s+\S+,\s+\[pc(?:,\s+#(-?\d+))?\]\s*@\s*([0-9a-fA-F]+)", line)
    if m:
        vldr_targets.add(int(m.group(3), 16))
    # Match ARM switch jump table dispatch: ldrls/ldrcc/ldr pc, [pc, rX, lsl #2]
    m_jt = re.search(r"^\s*([0-9a-fA-F]+):\s+[0-9a-fA-F]+\s+ldr\w*\s+pc,\s+\[pc,\s*\w+,\s*lsl\s*#2\]", line)
    if m_jt:
        jt_pc = int(m_jt.group(1), 16)
        # On ARM, [pc, rX, lsl #2] with rX=0 reads from jt_pc + 8
        jump_table_starts.add(jt_pc + 8)

reloc_addrs = set()
_, _, t_addr, _, t_size = sections[".text"]
t_end = t_addr + t_size
_, _, r_addr, _, r_size = sections[".rodata"]
r_end = r_addr + r_size

def is_non_text_ptr(val):
    # Points to .rodata, .ARM.extab, .ARM.exidx, or RW segment (.init_array, .got, .data, .bss)
    return (r_addr <= val < RX_END) or (RW_START <= val <= RW_END)

# 4. Collect pointers in .text ($d literal pools)
skipped_int_consts = []
for i, (addr, mtype) in enumerate(sec12_map):
    if mtype == "$d":
        end_addr = sec12_map[i+1][0] if i+1 < len(sec12_map) else t_end
        is_jump_table = (addr in jump_table_starts)
        for p in range(addr, end_addr, 4):
            if p in vldr_targets:
                continue
            val = get_u32(p)
            if is_non_text_ptr(val):
                reloc_addrs.add(p)
            elif PMIN <= val < t_end:
                # Only relocate .text values if this $d block is a switch jump table OR val is an exact function start!
                if is_jump_table or (val in func_addrs):
                    reloc_addrs.add(p)
                else:
                    skipped_int_consts.append((p, val))

print(f"Protected {len(skipped_int_consts)} integer constants in .text $d pools from false relocation!")
print("Sample protected constants:", [(hex(p), hex(v), v) for p, v in skipped_int_consts[:12]])

# 5. Collect pointers in .rodata (vtables, RTTI, string tables; skip mpg123/GL math tables)
RODATA_MATH_BLACKLIST = {
    ".rodata", "<anonymous>", "BlendFuncDstValues", "BlendFuncSrcValues",
    "_ZN2gr10FORMATDESCE", "_ZZN6Action6updateEbE5C.449", "_ZZN6Action6updateEbE5C.450",
    "alloc_0", "alloc_1", "alloc_2", "alloc_3", "alloc_4",
    "gainpow2", "intwinbase", "ispow", "layer12_table", "luaP_opmodes",
    "stab.6464", "tan1_1", "tan1_2", "tan2_1", "tblofs"
}
for sva, ssz, stype, sname in sec14_syms:
    if sname in RODATA_MATH_BLACKLIST:
        continue
    for p in range(sva, sva + ssz, 4):
        if r_addr <= p < r_end:
            val = get_u32(p)
            if (PMIN <= val < RX_END) or (RW_START <= val <= RW_END):
                reloc_addrs.add(p)


# 6. Collect pointers in .init_array, .fini_array, .got, and .data
for sname in [".init_array", ".fini_array", ".got"]:
    _, _, sa, _, ss = sections[sname]
    for p in range(sa, sa + ss, 4):
        val = get_u32(p)
        if (PMIN <= val < RX_END) or (RW_START <= val <= RW_END):
            reloc_addrs.add(p)

DATA_PTR_WHITELIST = {
    "BlendEquationStrings", "BlendFuncDstStrings", "BlendFuncSrcStrings",
    "CullFaceStrings", "DepthFuncStrings", "bad_reader", "delim",
    "ht", "htc", "mpg123_decoder_list", "pnts", "readers"
}
for sva, ssz, stype, sname in sec23_syms:
    if sname in DATA_PTR_WHITELIST:
        for p in range(sva, sva + ssz, 4):
            val = get_u32(p)
            if (PMIN <= val < RX_END) or (RW_START <= val <= RW_END):
                reloc_addrs.add(p)

# 7. Redirect all 310 PLT imports and .got entries directly to __wrap_<sym>
rel_out = subprocess.check_output(["arm-vita-eabi-readelf", "-W", "-r", ELF_PATH], text=True)
sym_ext_map = {}
in_plt = False
in_dyn = False

def patch_arm(vaddr, insns):
    for idx, insn in enumerate(insns):
        set_u32(vaddr + idx * 4, insn)
        reloc_addrs.discard(vaddr + idx * 4)

for line in rel_out.splitlines():
    if "'.rel.dyn'" in line:
        in_dyn, in_plt = True, False
        continue
    elif "'.rel.plt'" in line:
        in_dyn, in_plt = False, True
        continue
    m = re.match(r"^([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+(\S+)\s+([0-9a-fA-F]+)\s+(.*)$", line.strip())
    if m:
        off, rtype, sval, sname = m.groups()
        off = int(off, 16)
        sval = int(sval, 16)
        clean_name = sname.split("@")[0].strip()
        if in_plt and rtype == "R_ARM_JUMP_SLOT":
            sym_ext_map[off] = f"__wrap_{clean_name}"
            reloc_addrs.discard(off)
            if sval != 0:
                patch_arm(sval, [0xe51ff004, 0, 0xe12fff1e])
                sym_ext_map[sval + 4] = f"__wrap_{clean_name}"
            else:
                patch_arm(0x0000d640, [0xe12fff1e, 0xe12fff1e, 0xe12fff1e])
        elif in_dyn and rtype == "R_ARM_GLOB_DAT":
            if sval == 0:
                sym_ext_map[off] = f"__wrap_{clean_name}"
                reloc_addrs.discard(off)
            else:
                set_u32(off, sval)
                reloc_addrs.add(off)

# 8. Hook N900 Accelerometer methods directly to Vita motion/stick handler
patch_arm(0x0d38a8, [0xe3a00001, 0xe12fff1e]) # Accelerometer::start -> return 1
patch_arm(0x0d3834, [0xe3a00000, 0xe12fff1e]) # Accelerometer::stop  -> return 0
patch_arm(0x0d3320, [0xe51ff004, 0])          # Accelerometer::getDataFiltered
patch_arm(0x0d3358, [0xe51ff004, 0])          # Accelerometer::getData
sym_ext_map[0x0d3324] = "__wrap_vita_accel_get_data_filtered"   # low-pass filtered vector
sym_ext_map[0x0d335c] = "__wrap_vita_accel_get_data_raw"        # raw vector (jump spikes)

# 9. Emit bounce_image.S and compile bounce_reloc.o
def target_expr(val):
    if RX_START <= val <= RX_END:
        return f"(bounce_rx_base + {val - RX_START})"
    elif RW_START <= val <= RW_END:
        return f"(bounce_rw_base + {val - RW_START})"
    raise ValueError(f"Bad target {val:#x}")

with open("bounce_rx.bin", "wb") as bf:
    bf.write(rx_img)
with open("bounce_rw.bin", "wb") as bf:
    bf.write(rw_img)

with open("bounce_image.S", "w") as out:
    out.write(".syntax unified\n.arm\n\n")
    out.write(".section .text.bounce, \"ax\", %progbits\n")
    out.write(".balign 32\n")
    out.write(".global bounce_rx_base\n")
    out.write("bounce_rx_base:\n")

    rx_special = sorted(set([a for a in reloc_addrs if RX_START <= a < RX_END] + [a for a in sym_ext_map if RX_START <= a < RX_END]))
    cur = RX_START
    for addr in rx_special:
        if addr > cur:
            out.write(f"  .incbin \"bounce_rx.bin\", {cur - RX_START}, {addr - cur}\n")
        if addr in sym_ext_map:
            out.write(f"  .word {sym_ext_map[addr]}\n")
        else:
            out.write(f"  .word {target_expr(get_u32(addr))}\n")
        cur = addr + 4
    if cur < RX_END:
        out.write(f"  .incbin \"bounce_rx.bin\", {cur - RX_START}, {RX_END - cur}\n")

    out.write(f"\n.global bounce_main\n.type bounce_main, %function\n.set bounce_main, bounce_rx_base + {0x0c32ac - RX_START}\n")
    out.write(f".global bounce_x11_state\n.set bounce_x11_state, bounce_rw_base + {0x001d14a0 - RW_START}\n")

    out.write("\n.section .data.bounce, \"aw\", %progbits\n")
    out.write(".balign 32\n")
    out.write(".global bounce_rw_base\n")
    out.write("bounce_rw_base:\n")

    rw_special = sorted(set([a for a in reloc_addrs if RW_START <= a < DATA_END] + [a for a in sym_ext_map if RW_START <= a < DATA_END]))
    cur = RW_START
    for addr in rw_special:
        if addr > cur:
            out.write(f"  .incbin \"bounce_rw.bin\", {cur - RW_START}, {addr - cur}\n")
        if addr in sym_ext_map:
            out.write(f"  .word {sym_ext_map[addr]}\n")
        else:
            out.write(f"  .word {target_expr(get_u32(addr))}\n")
        cur = addr + 4
    if cur < DATA_END:
        out.write(f"  .incbin \"bounce_rw.bin\", {cur - RW_START}, {DATA_END - cur}\n")
    out.write(f"  .space {RW_END - DATA_END}, 0\n")

print("Assembling bounce_reloc.o...")
subprocess.check_call(["arm-vita-eabi-gcc", "-c", "bounce_image.S", "-o", "bounce_reloc.o"])
print("Compiled bounce_reloc.o successfully!")
