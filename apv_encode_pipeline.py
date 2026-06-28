#!/usr/bin/env python3
"""
apv_encode_pipeline.py
----------------------
Complete APV encoding pipeline producing 4 output files:

  fdct_output.txt      — FDCT coefficients          (already generated)
  quant_output.txt     — quantised coefficients
  zigzag_output.txt    — zigzag-scanned coefficients
  encoded_output.txt   — run-level encoded stream

encoded_output.txt format (one symbol per line, 3 columns):
  TYPE  VALUE  SIGN
  TYPE: 0=DC_DIFF  1=RUN  2=LEVEL  3=EOB
  VALUE: abs value of the symbol
  SIGN:  0=positive  1=negative  (always 0 for RUN and EOB)

Blank line between blocks in all files.

Usage:
  python3 apv_encode_pipeline.py
  python3 apv_encode_pipeline.py --qp 32 --seed 42
"""

import argparse
import os
import random

# -- APV transform matrix T (RFC 9924 §6.3.2.3) ------------------------------
T = [
    [ 64,  64,  64,  64,  64,  64,  64,  64],
    [ 89,  75,  50,  18, -18, -50, -75, -89],
    [ 84,  35, -35, -84, -84, -35,  35,  84],
    [ 75, -18, -89, -50,  50,  89,  18, -75],
    [ 64, -64, -64,  64,  64, -64, -64,  64],
    [ 50, -89,  18,  75, -75, -18,  89, -50],
    [ 35, -84,  84, -35, -35,  84, -84,  35],
    [ 18, -50,  75, -89,  89, -75,  50, -18],
]

# -- APV level scale table (RFC 9924 §6.3.3) ---------------------------------
LEVEL_SCALE = [40, 45, 51, 57, 64, 72]
QM_FLAT     = [[16]*8 for _ in range(8)]


# -- FDCT ---------------------------------------------------------------------
def fdct_shift1(bit_depth): return bit_depth - 6   # row pass: 4 for 10-bit
def fdct_shift2():          return 9               # column pass: constant

def fdct_1d(x, shift):
    """8-point FDCT butterfly pass with correct per-pass shift (RFC 9924)."""
    e = [x[k] + x[7-k] for k in range(4)]
    o = [x[k] - x[7-k] for k in range(4)]
    ee0, eo0 = e[0]+e[3], e[0]-e[3]
    ee1, eo1 = e[1]+e[2], e[1]-e[2]
    add = 1 << (shift-1)
    out = [0]*8
    out[0] = (T[0][0]*ee0 + T[0][1]*ee1 + add) >> shift
    out[4] = (T[4][0]*ee0 + T[4][1]*ee1 + add) >> shift
    out[2] = (T[2][0]*eo0 + T[2][1]*eo1 + add) >> shift
    out[6] = (T[6][0]*eo0 + T[6][1]*eo1 + add) >> shift
    for row in (1, 3, 5, 7):
        out[row] = (sum(T[row][m]*o[m] for m in range(4)) + add) >> shift
    return out

def fdct_2d(block, bit_depth=10):
    """Full 8x8 2D FDCT. Returns result in hardware-natural order (no final
    transpose): result[c] is the column-pass output for column c."""
    s1, s2 = fdct_shift1(bit_depth), fdct_shift2()
    row_out = [fdct_1d(block[r], s1) for r in range(8)]
    cols    = [[row_out[r][c] for r in range(8)] for c in range(8)]
    return  [fdct_1d(cols[c], s2) for c in range(8)]


# -- Quantisation (RFC 9924 §6.3.3) ------------------------------------------
def quantise(block, qp, bit_depth=10, qm=None):
    if qm is None: qm = QM_FLAT
    scale   = LEVEL_SCALE[qp % 6]
    shift   = qp // 6
    bdShift = bit_depth - 2
    result  = []
    for r in range(8):
        row = []
        for c in range(8):
            coeff = block[r][c]
            step  = (qm[r][c] * scale * (1 << shift)) >> bdShift
            if step == 0:
                row.append(0)
            elif coeff >= 0:
                row.append( (coeff + step//2) // step)
            else:
                row.append(-( (-coeff + step//2) // step))
        result.append(row)
    return result


# -- Zigzag scan ---------------------------------------------------------------
def _gen_zigzag():
    scan = []
    for d in range(15):
        if d % 2 == 0:
            r = min(d, 7); c = d - r
            while r >= 0 and c <= 7:
                scan.append(r*8+c); r -= 1; c += 1
        else:
            c = min(d, 7); r = d - c
            while c >= 0 and r <= 7:
                scan.append(r*8+c); r += 1; c -= 1
    return scan

SCAN = _gen_zigzag()

def zigzag_scan(block):
    flat = [block[r][c] for r in range(8) for c in range(8)]
    return [flat[i] for i in SCAN]


# -- APV h(v) VLC encoder (RFC 9924 §6.4.3) -----------------------------------
def encode_hv(value, kParam):
    """
    Encode a non-negative integer using the APV h(v) variable length code.
    Returns the codeword as a binary string (magnitude only, no sign bit).
    """
    k         = kParam
    short_max  = (1 << k) - 1          # values 0 .. 2^k-1
    medium_max = (1 << (k+1)) - 1      # values 2^k .. 2^(k+1)-1

    if value <= short_max:
        # SHORT: prefix '1' + k-bit suffix
        bits = '1'
        if k > 0:
            bits += format(value, f'0{k}b')

    elif value <= medium_max:
        # MEDIUM: prefix '00' + k-bit suffix
        bits = '00'
        if k > 0:
            bits += format(value - (1 << k), f'0{k}b')

    else:
        # EXTENDED: prefix '01' + extension zeros + stop '1' + k-bit suffix
        sym  = 2 << k
        bits = '01'
        kk   = k
        while sym + (1 << kk) <= value:
            sym  += (1 << kk)
            kk   += 1
            bits += '0'
        bits += '1'                     # stop bit
        remainder = value - sym
        if kk > 0:
            bits += format(remainder, f'0{kk}b')

    return bits


def clip_kp(value, rsh, hi):
    return max(0, min(hi, value >> rsh))
# Symbol types
DC    = 0
RUN   = 1
LEVEL = 2
EOB   = 3

def encode_block(zz_coeffs, prev_dc, kp_dc, kp_run, kp_ac):
    """
    Encode one zigzag-ordered 64-element coefficient list.
    Returns list of (type, value, sign, kparam, codeword) tuples
    and updated (prev_dc, kp_dc, kp_run, kp_ac).

    TYPE  VALUE                       SIGN  KPARAM  CODEWORD
    DC    abs(current_dc - prev_dc)   0/1   kp_dc   magnitude + sign bits
    RUN   zero count before level     0     kp_run  magnitude bits
    LVL   abs(coeff) - 1              0/1   kp_ac  magnitude + sign bits
    EOB   0                           0     kp_run  magnitude bits
    """
    symbols = []

    # DC coefficient
    current_dc = zz_coeffs[0]
    dc_diff    = current_dc - prev_dc
    abs_dc_coeff_diff     = abs(dc_diff)
    sign_dc_coeff_diff    = 1 if dc_diff < 0 else 0
    cw_dc_diff      = encode_hv(abs_dc_coeff_diff, kp_dc)
    if abs_dc_coeff_diff != 0:
        cw_dc_diff += str(sign_dc_coeff_diff)           # sign bit appended after magnitude
    symbols.append((DC, abs_dc_coeff_diff, sign_dc_coeff_diff, kp_dc, cw_dc_diff))
    kp_dc = clip_kp(abs_dc_coeff_diff, 1, 5)      # update kParam_DC

    # AC coefficients
    run          = 0
    last_nonzero = -1
    for pos in range(1, 64):
        if zz_coeffs[pos] != 0:
            last_nonzero = pos

    for pos in range(1, 64):
        coeff = zz_coeffs[pos]
        if coeff == 0:
            run += 1
        else:
            # RUN symbol
            cw_run = encode_hv(run, kp_run)
            symbols.append((RUN, run, 0, kp_run, cw_run))
            kp_run = clip_kp(run, 2, 2)

            # LEVEL symbol  (abs_ac_coeff_minus1 per RFC 9924)
            abs_coeff      = abs(coeff)           # actual magnitude
            abs_ac_coeff_min1 = abs_coeff - 1    # bitstream value (min 0)
            sign_ac           = 1 if coeff < 0 else 0
            cw_ac             = encode_hv(abs_ac_coeff_min1, kp_ac)
            cw_ac            += str(sign_ac)         # sign bit always present for non-zero AC
            symbols.append((LEVEL, abs_ac_coeff_min1, sign_ac, kp_ac, cw_ac))
            kp_ac = clip_kp(abs_ac_coeff_min1, 2, 4)
            run    = 0

    # EOB — trailing zeros from last nonzero to end of block
    if last_nonzero < 63:
        # 'run' already holds the exact trailing-zero count accumulated since
        # the last nonzero coefficient; no further arithmetic needed.
        eob_run = run if last_nonzero >= 0 else 63
        cw_eob  = encode_hv(eob_run, kp_run)
        symbols.append((EOB, eob_run, 0, kp_run, cw_eob))

    return symbols, current_dc, kp_dc, kp_run, kp_ac


# -- Compression rate ----------------------------------------------------------
def block_compression_rate(symbols, bit_depth=10):
    """
    Compute compression statistics for one encoded block.

    Returns a dict with:
      raw_bits     — bits needed to store the original 8x8 block uncompressed
      coded_bits   — actual bits used by the h(v) codewords (incl. sign bits)
      ratio        — raw_bits / coded_bits  (>1 means compression achieved)
      bpp          — coded bits per pixel (coded_bits / 64)
      zeros        — count of LEVEL symbols that are zero (always 0, kept for clarity)
      num_symbols  — total symbols (DC + RUN + LEVEL + EOB)
    """
    raw_bits   = 64 * bit_depth                  # uncompressed size of 8x8 block
    coded_bits = sum(len(cw) for _, _, _, _, cw in symbols)

    ratio = raw_bits / coded_bits if coded_bits > 0 else float('inf')
    bpp   = coded_bits / 64.0

    return {
        "raw_bits":    raw_bits,
        "coded_bits":  coded_bits,
        "ratio":       ratio,
        "bpp":         bpp,
        "num_symbols": len(symbols),
    }
def special_cases():
    MAX = 511; MIN = -512
    cases = []
    cases.append([MAX]*8)
    cases.append([MIN]*8)
    for i in range(8):
        v=[0]*8; v[i]=128; cases.append(v)
    for n in range(8):
        d=[MAX if T[k][n]>0 else MIN if T[k][n]<0 else 0 for k in range(8)]
        cases.append(d)
    cases.append([MAX if i%2==0 else MIN for i in range(8)])
    cases.append([MIN if i%2==0 else MAX for i in range(8)])
    cases.append([0]*8)
    cases.append([MIN + i*(MAX-MIN)//7 for i in range(8)])
    cases.append([MAX - i*(MAX-MIN)//7 for i in range(8)])
    return cases


# -- Write helpers -------------------------------------------------------------
def write_8x8(f, block, fmt):
    """Write 8x8 block as 8 lines of 8 values, then blank line."""
    for row in block:
        f.write(" ".join(fmt(v) for v in row) + "\n")
    f.write("\n")

def write_1d_as_8rows(f, flat, fmt):
    """Write 64-element list as 8 lines of 8 values, then blank line."""
    for row in range(8):
        f.write(" ".join(fmt(flat[row*8+c]) for c in range(8)) + "\n")
    f.write("\n")


# -- Main ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rows",      type=int,  default=100)
    parser.add_argument("--smooth",    action="store_true",
                        help="Generate smooth (image-like) blocks instead of pure random")
    parser.add_argument("--smooth_var",type=int,  default=8,
                        help="Pixel variation around DC for smooth blocks (default 8)")
    parser.add_argument("--seed",      type=int,  default=None)
    parser.add_argument("--qp",        type=int,  default=22)
    parser.add_argument("--bit_depth", type=int,  default=10)
    parser.add_argument("--in_file",   default="fdct_input.txt")
    parser.add_argument("--fdct_file", default="fdct_output.txt")
    parser.add_argument("--qout_file", default="quant_output.txt")
    parser.add_argument("--zzout_file",default="zigzag_output.txt")
    parser.add_argument("--enc_file",  default="encoded_output.txt")
    parser.add_argument("--bits_file", default="bitstream_input.txt")
    parser.add_argument("--stats_file",default="compression_stats.txt")
    parser.add_argument("--pad",       action="store_true", default=True,
                        help="Pad last 32-bit word with zeros (default True)")
    args = parser.parse_args()

    rng         = random.Random(args.seed)
    actual_seed = args.seed if args.seed is not None else rng.randint(0, 2**32)
    rng         = random.Random(actual_seed)

    MIN10 = -512; MAX10 = 511

    # Generate rows
    if args.smooth:
        # Image-like data: each 8x8 block has a slowly varying base level
        # with small random variation — concentrates energy in low frequencies
        random_rows = []
        block_base  = rng.randint(MIN10//2, MAX10//2)
        for i in range(args.rows):
            if i % 8 == 0:
                # New block: drift the base level slightly
                block_base += rng.randint(-20, 20)
                block_base  = max(MIN10, min(MAX10, block_base))
            row = [max(MIN10, min(MAX10,
                       block_base + rng.randint(-args.smooth_var, args.smooth_var)))
                   for _ in range(8)]
            random_rows.append(row)
    else:
        random_rows = [[rng.randint(MIN10, MAX10) for _ in range(8)]
                       for _ in range(args.rows)]
    all_rows = random_rows + special_cases()
    total    = len(all_rows)

    # Group into 8x8 blocks (discard remainder)
    num_blocks = total // 8
    blocks     = [all_rows[b*8:(b+1)*8] for b in range(num_blocks)]

    # -- Open all output files -------------------------------------------------
    f_in   = open(args.in_file,    "w")
    f_fdct = open(args.fdct_file,  "w")
    f_qout = open(args.qout_file,  "w")
    f_zz   = open(args.zzout_file, "w")
    f_enc  = open(args.enc_file,   "w")

    bitstream   = ""   # accumulate all codeword bits here
    block_stats = []   # per-block compression stats

    prev_dc = 0   # DC predictor resets at tile start
    kp_dc   = 0   # initial kParam_DC: clip(0, 5, PrevDcDiff>>1) with PrevDcDiff=0 → 0
    kp_run  = 0   # initial kParam_run
    kp_ac  = 0   # initial kParam_lvl

    for b, block in enumerate(blocks):

        # -- Write input (4 samples per line) ---------------------------------
        for row in block:
            f_in.write(" ".join(f"{v:6d}" for v in row[:4]) + "\n")
            f_in.write(" ".join(f"{v:6d}" for v in row[4:]) + "\n")
        f_in.write("\n")

        # -- FDCT --------------------------------------------------------------
        dct_block = fdct_2d(block, args.bit_depth)
        write_8x8(f_fdct, dct_block, lambda v: f"{v:7d}")

        # -- Quantise ----------------------------------------------------------
        q_block = quantise(dct_block, args.qp, args.bit_depth)
        write_8x8(f_qout, q_block, lambda v: f"{v:6d}")

        # -- Zigzag scan -------------------------------------------------------
        zz = zigzag_scan(q_block)
        write_1d_as_8rows(f_zz, zz, lambda v: f"{v:6d}")

        # -- Run-level encode with codewords -----------------------------------
        symbols, prev_dc, kp_dc, kp_run, kp_ac = encode_block(zz, prev_dc, kp_dc, kp_run, kp_ac)

        for sym_type, value, sign, kparam, codeword in symbols:
            # For LEVEL symbols, value = abs(coeff)-1, so restore actual coeff for display
            if sym_type == LEVEL:
                actual = (value + 1) * (-1 if sign == 1 else 1)
            else:
                actual = -value if sign == 1 else value
            f_enc.write(f"{sym_type:2d}  {value:6d}  {sign:1d}  {actual:7d}  "
                        f"{kparam:1d}  {len(codeword):2d}  {codeword}\n")
            bitstream += codeword       # accumulate bits
        f_enc.write("\n")

        # -- Compression stats for this block ---------------------------------
        stats = block_compression_rate(symbols, args.bit_depth)
        block_stats.append(stats)

        # kParam_run and kParam_lvl reset at each block start
        kp_run = 0
        kp_ac = 0

    # Close all files
    for f in [f_in, f_fdct, f_qout, f_zz, f_enc]:
        f.close()

    # -- Write bitstream file — 32 bits per line -------------------------------
    total_bits = len(bitstream)
    remainder  = total_bits % 32

    if remainder != 0:
        if args.pad:
            bitstream += '0' * (32 - remainder)   # pad last word with zeros
        else:
            bitstream  = bitstream[:total_bits - remainder]  # drop incomplete word

    with open(args.bits_file, "w") as f:
        for i in range(0, len(bitstream), 32):
            f.write(bitstream[i:i+32] + "\n")

    # -- Write compression stats file ------------------------------------------
    with open(args.stats_file, "w") as f:
        f.write(f"{'Block':>6} {'RawBits':>8} {'CodedBits':>10} "
                f"{'Ratio':>8} {'BPP':>6} {'Symbols':>8}\n")
        for b, s in enumerate(block_stats):
            f.write(f"{b:6d} {s['raw_bits']:8d} {s['coded_bits']:10d} "
                    f"{s['ratio']:8.3f} {s['bpp']:6.3f} {s['num_symbols']:8d}\n")

        total_raw   = sum(s['raw_bits']   for s in block_stats)
        total_coded = sum(s['coded_bits'] for s in block_stats)
        f.write(f"\n{'TOTAL':>6} {total_raw:8d} {total_coded:10d} "
                f"{total_raw/total_coded:8.3f} "
                f"{total_coded/(64*len(block_stats)):6.3f} "
                f"{sum(s['num_symbols'] for s in block_stats):8d}\n")

    # -- Print summary ---------------------------------------------------------
    print(f"Seed:      {actual_seed}")
    print(f"Blocks:    {num_blocks}  ({args.rows} random rows → {num_blocks} 8x8 blocks)")
    print(f"qP:        {args.qp}  bit_depth={args.bit_depth}")
    print()
    type_names = {DC:"DC", RUN:"RUN", LEVEL:"LVL", EOB:"EOB"}
    print(f"encoded_output.txt format:")
    print(f"  TYPE  VALUE  SIGN  SIGNED_VALUE  KPARAM  CW_LEN  CODEWORD")
    print(f"  0=DC_DIFF  1=RUN  2=LEVEL  3=EOB")
    print(f"  SIGN:    0=positive  1=negative  (always 0 for RUN and EOB)")
    print(f"  KPARAM:  kParam used to encode this symbol")
    print(f"  CODEWORD: h(v) binary codeword (magnitude + sign bit if applicable)")
    print()
    for fn in [args.in_file, args.fdct_file, args.qout_file,
               args.zzout_file, args.enc_file, args.bits_file, args.stats_file]:
        print(f"  {fn:<25} {os.path.getsize(fn):>8,} bytes")

    words     = len(bitstream) // 32
    last_pad  = (32 - total_bits % 32) % 32
    print()
    print(f"  Total bits:   {total_bits}")
    print(f"  32-bit words: {words}")
    print(f"  Padding:      {last_pad} zero bits added to last word"
          if last_pad else "  Last word:   exact 32-bit boundary, no padding")
    print()
    print(f"Compression summary:")
    total_raw   = sum(s['raw_bits']   for s in block_stats)
    total_coded = sum(s['coded_bits'] for s in block_stats)
    print(f"  Total raw bits:    {total_raw}  ({total_raw//8} bytes)")
    print(f"  Total coded bits:  {total_coded}  ({total_coded//8} bytes)")
    print(f"  Overall ratio:     {total_raw/total_coded:.3f}x")
    print(f"  Avg bits/pixel:    {total_coded/(64*len(block_stats)):.3f}  "
          f"(raw = {args.bit_depth}.000)")
    print()
    print(f"  Per-block ratio range: "
          f"{min(s['ratio'] for s in block_stats):.3f}x "
          f"to {max(s['ratio'] for s in block_stats):.3f}x")


if __name__ == "__main__":
    main()
