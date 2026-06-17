#!/usr/bin/env python3
"""
apv_fdct_butterfly_ref.py
--------------------------
APV (ISO/IEC 23094-10 / RFC 9924) forward/inverse 8-point DCT, fast butterfly
form (E/O/EE/EO decomposition, ~28 multiplies instead of 64).

Shift amounts are bit-depth dependent and asymmetric between the row and
column pass (NOT a uniform (acc+64)>>7 applied twice). Rounding is applied
once per pass, exactly as the codec does, so the total FDCT+IDCT shift is
always 30 regardless of bit depth.

Functions:
    fdct_1d(x, shift)            8-point forward butterfly, one row/column
    fdct_2d(block, bit_depth)    full 8x8 forward transform
    idct_1d(x, shift)            8-point inverse butterfly, one row/column
    idct_2d(block, bit_depth)    full 8x8 inverse transform
    fdct_shift1(bit_depth)       row-pass shift  (forward)
    fdct_shift2(bit_depth)       column-pass shift (forward)
    idct_shift1(bit_depth)       pass-1 shift (inverse, fixed at 7)
    idct_shift2(bit_depth)       pass-2 shift (inverse)
"""

import argparse
import random

# ── Transform matrix (RFC 9924) ─────────────────────────────────────────────
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


def clip16(v):
    return max(-32768, min(32767, v))


# ── Shift formulas (bit-depth dependent) ────────────────────────────────────
def fdct_shift1(bit_depth):
    return bit_depth - 6          # row pass


def fdct_shift2(bit_depth):
    return 9                      # column pass, constant


def idct_shift1(bit_depth):
    return 7                      # pass 1, fixed


def idct_shift2(bit_depth):
    return 12 - (bit_depth - 8)   # pass 2


# ── Forward DCT, 8-point butterfly ──────────────────────────────────────────
def fdct_1d(x, shift):
    """One 8-point FDCT butterfly pass. x: 8 samples. Returns 8 coefficients."""
    e = [x[k] + x[7-k] for k in range(4)]
    o = [x[k] - x[7-k] for k in range(4)]
    ee0, eo0 = e[0]+e[3], e[0]-e[3]
    ee1, eo1 = e[1]+e[2], e[1]-e[2]

    add = 1 << (shift-1)
    out = [0]*8
    out[0] = clip16((T[0][0]*ee0 + T[0][1]*ee1 + add) >> shift)
    out[4] = clip16((T[4][0]*ee0 + T[4][1]*ee1 + add) >> shift)
    out[2] = clip16((T[2][0]*eo0 + T[2][1]*eo1 + add) >> shift)
    out[6] = clip16((T[6][0]*eo0 + T[6][1]*eo1 + add) >> shift)
    for row in (1, 3, 5, 7):
        out[row] = clip16((sum(T[row][m]*o[m] for m in range(4)) + add) >> shift)
    return out


def fdct_2d(block, bit_depth=10):
    """Full 2D forward transform. block: 8x8 list, row-major. Returns 8x8 list."""
    shift1 = fdct_shift1(bit_depth)
    shift2 = fdct_shift2(bit_depth)

    row_out = [fdct_1d(block[r], shift1) for r in range(8)]
    cols = [[row_out[r][c] for r in range(8)] for c in range(8)]
    col_out = [fdct_1d(cols[c], shift2) for c in range(8)]

    result = [[0]*8 for _ in range(8)]
    for c in range(8):
        for m in range(8):
            result[m][c] = col_out[c][m]
    return result


# ── Inverse DCT, 8-point butterfly ──────────────────────────────────────────
def idct_1d(d, shift):
    """One 8-point IDCT butterfly pass. d: 8 coefficients. Returns 8 samples."""
    o = [sum(T[k][m]*d[k] for k in (1, 3, 5, 7)) for m in range(4)]
    eo0 = T[2][0]*d[2] + T[6][0]*d[6]
    eo1 = T[2][1]*d[2] + T[6][1]*d[6]
    ee0 = T[0][0]*d[0] + T[4][0]*d[4]
    ee1 = T[0][1]*d[0] + T[4][1]*d[4]

    e = [0]*4
    e[0], e[3] = ee0+eo0, ee0-eo0
    e[1], e[2] = ee1+eo1, ee1-eo1

    add = 1 << (shift-1)
    out = [0]*8
    for k in range(4):
        out[k]   = clip16((e[k]   + o[k]   + add) >> shift)
        out[k+4] = clip16((e[3-k] - o[3-k] + add) >> shift)
    return out


def idct_2d(block, bit_depth=10):
    """Full 2D inverse transform. block: 8x8 list, row-major (freq domain)."""
    shift1 = idct_shift1(bit_depth)
    shift2 = idct_shift2(bit_depth)

    pass1 = [idct_1d(block[r], shift1) for r in range(8)]
    cols = [[pass1[r][c] for r in range(8)] for c in range(8)]
    pass2 = [idct_1d(cols[c], shift2) for c in range(8)]

    result = [[0]*8 for _ in range(8)]
    for c in range(8):
        for m in range(8):
            result[m][c] = pass2[c][m]
    return result


# ── Test vectors / verification ─────────────────────────────────────────────
def special_blocks(bit_depth):
    hi = (1 << (bit_depth-1)) - 1
    lo = -(1 << (bit_depth-1))
    blocks = [[[hi]*8 for _ in range(8)], [[lo]*8 for _ in range(8)],
              [[0]*8 for _ in range(8)]]
    for i in range(8):
        b = [[0]*8 for _ in range(8)]
        b[i // 8][i % 8] = hi
        blocks.append(b)
    blocks.append([[hi if (r+c) % 2 == 0 else lo for c in range(8)] for r in range(8)])
    return blocks


def write_8x8(f, block, fmt):
    for row in block:
        f.write(" ".join(fmt(v) for v in row) + "\n")
    f.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bit_depth", type=int, default=10)
    parser.add_argument("--blocks",    type=int, default=50)
    parser.add_argument("--seed",      type=int, default=42)
    parser.add_argument("--gen_vectors", action="store_true")
    parser.add_argument("--in_file",  default="fdct_butterfly_input.txt")
    parser.add_argument("--out_file", default="fdct_butterfly_output.txt")
    args = parser.parse_args()

    bit_depth = args.bit_depth
    rng = random.Random(args.seed)
    hi = (1 << (bit_depth-1)) - 1
    lo = -(1 << (bit_depth-1))

    s1, s2 = fdct_shift1(bit_depth), fdct_shift2(bit_depth)
    i1, i2 = idct_shift1(bit_depth), idct_shift2(bit_depth)

    print(f"=== Shifts for bit_depth={bit_depth} ===")
    print(f"  FDCT: shift1={s1}  shift2={s2}  (total {s1+s2})")
    print(f"  IDCT: shift1={i1}  shift2={i2}  (total {i1+i2})")
    print(f"  Combined total = {s1+s2+i1+i2}  (always 30, by design)")

    max_err = 0
    for _ in range(500):
        block = [[rng.randint(lo, hi) for _ in range(8)] for _ in range(8)]
        coeffs = fdct_2d(block, bit_depth)
        recon  = idct_2d(coeffs, bit_depth)
        err = max(abs(block[r][c]-recon[r][c]) for r in range(8) for c in range(8))
        max_err = max(max_err, err)
    print()
    print(f"Round-trip max error (fdct_2d -> idct_2d, no quantisation): {max_err}")

    if args.gen_vectors:
        blocks = [[[rng.randint(lo, hi) for _ in range(8)] for _ in range(8)]
                  for _ in range(args.blocks)]
        blocks += special_blocks(bit_depth)

        with open(args.in_file, "w") as f_in, open(args.out_file, "w") as f_out:
            for block in blocks:
                write_8x8(f_in, block, lambda v: f"{v:6d}")
                coeffs = fdct_2d(block, bit_depth)
                write_8x8(f_out, coeffs, lambda v: f"{v:7d}")

        print()
        print(f"Test vectors written: {args.in_file}, {args.out_file}  "
              f"({len(blocks)} blocks, bit_depth={bit_depth})")


if __name__ == "__main__":
    main()
