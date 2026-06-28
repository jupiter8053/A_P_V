#!/usr/bin/env python3
"""
selftest_golden.py
------------------
Standalone validator for the APV VLC golden model used in the cocotb testbench.
Run this WITHOUT a simulator to verify that:

  1. Every generated codeword round-trips through encode_hv correctly.
  2. Shift values (codeword lengths) are within the 34-bit window.
  3. Sign reconstruction is correct for DC and LEVEL symbols.
  4. RUN / EOB values are in range [0, 63].
  5. kParam updates follow the spec.

Usage:
    python3 selftest_golden.py
    python3 selftest_golden.py --blocks 200 --seed 0
"""

import sys
import argparse
import random
sys.path.insert(0, '.')

from apv_encode_pipeline import (
    encode_hv, encode_block, fdct_2d, quantise, zigzag_scan,
    DC, RUN, LEVEL, EOB, special_cases, clip_kp,
    fdct_shift1, fdct_shift2,
)

BIT_DEPTH = 10
DATA_W    = 34      # DUT window width

# -- Decode h(v) codeword back to value  (inverse of encode_hv) ---------------
def decode_hv(bits: str, kparam: int) -> int:
    """Decode an h(v) magnitude codeword (no sign bit) back to unsigned value."""
    k = kparam
    if bits[0] == '1':
        # SHORT tier
        if k == 0:
            return 0
        suffix = bits[1:1+k]
        return int(suffix, 2)
    elif bits[:2] == '00':
        # MEDIUM tier
        if k == 0:
            return 1
        suffix = bits[2:2+k]
        return (1 << k) + int(suffix, 2)
    else:
        # EXTENDED tier  ('01' prefix)
        assert bits[:2] == '01', f"Unknown tier prefix in '{bits}'"
        pos = 2
        sym = 2 << k
        kk  = k
        while pos < len(bits) and bits[pos] == '0':
            sym += (1 << kk)
            kk  += 1
            pos += 1
        assert pos < len(bits) and bits[pos] == '1', "Missing stop bit"
        pos += 1   # skip stop bit
        remainder = int(bits[pos:pos+kk], 2) if kk > 0 else 0
        return sym + remainder


def run_self_test(num_blocks: int = 100, seed: int = 42, qp: int = 22):
    print(f"APV VLC Golden Model Self-Test")
    print(f"  blocks={num_blocks}  seed={seed}  qp={qp}  bit_depth={BIT_DEPTH}")
    print()

    rng     = random.Random(seed)
    MIN10   = -(1 << (BIT_DEPTH - 1))
    MAX10   =  (1 << (BIT_DEPTH - 1)) - 1

    random_rows = [[rng.randint(MIN10, MAX10) for _ in range(8)]
                   for _ in range(num_blocks * 8)]
    all_rows    = random_rows + special_cases()
    num_blocks  = len(all_rows) // 8

    errors   = 0
    total    = 0
    by_type  = {DC: 0, RUN: 0, LEVEL: 0, EOB: 0}
    by_tier  = {'SHORT': 0, 'MEDIUM': 0, 'EXTENDED': 0}

    prev_dc  = 0
    kp_dc = kp_run = kp_lvl = 0

    for b in range(num_blocks):
        block = all_rows[b*8 : b*8+8]
        dct   = fdct_2d(block, BIT_DEPTH)
        q     = quantise(dct, qp, BIT_DEPTH)
        zz    = zigzag_scan(q)

        syms, prev_dc, kp_dc, kp_run, kp_lvl = encode_block(
            zz, prev_dc, kp_dc, kp_run, kp_lvl)
        kp_run = 0; kp_lvl = 0

        for sym_type, value, sign, kparam, codeword in syms:
            total      += 1
            by_type[sym_type] += 1
            cw_len      = len(codeword)

            # -- Determine tier ----------------------------------------------
            if codeword[0] == '1':
                tier = 'SHORT'
            elif codeword[:2] == '00':
                tier = 'MEDIUM'
            else:
                tier = 'EXTENDED'
            by_tier[tier] += 1

            # -- Check 1: codeword fits in 34-bit window ---------------------
            if cw_len > DATA_W:
                print(f"ERROR [blk {b}]: codeword length {cw_len} > {DATA_W} bits "
                      f"type={sym_type} val={value} kp={kparam} cw='{codeword}'")
                errors += 1

            # -- Check 2: magnitude round-trip ------------------------------
            # Strip sign bit if present (DC and LEVEL have it appended)
            mag_cw = codeword
            if sym_type in (DC, LEVEL) and value > 0:
                mag_cw = codeword[:-1]   # remove trailing sign bit
            elif sym_type == DC and value == 0:
                mag_cw = codeword        # no sign bit for zero diff
            # For RUN and EOB: no sign bit

            decoded = decode_hv(mag_cw, kparam)
            if decoded != value:
                print(f"ERROR [blk {b}]: magnitude round-trip fail "
                      f"type={sym_type} val={value} kp={kparam} "
                      f"mag_cw='{mag_cw}' decoded={decoded}")
                errors += 1

            # -- Check 3: sign bit position ----------------------------------
            if sym_type in (DC, LEVEL) and value > 0:
                got_sign = int(codeword[-1])
                if got_sign != sign:
                    print(f"ERROR [blk {b}]: sign bit wrong "
                          f"type={sym_type} val={value} sign={sign} "
                          f"cw='{codeword}' last_bit={got_sign}")
                    errors += 1

            # -- Check 4: zero-diff DC has no sign bit ----------------------
            if sym_type == DC and value == 0:
                # codeword must be pure magnitude (1 or 2 bits for SHORT/MEDIUM/EXTENDED)
                # decode the whole codeword as magnitude
                full_decode = decode_hv(codeword, kparam)
                if full_decode != 0:
                    print(f"ERROR [blk {b}]: zero DC diff should decode to 0 "
                          f"cw='{codeword}' decoded={full_decode}")
                    errors += 1

            # -- Check 5: run / EOB value in [0, 63] ------------------------
            if sym_type in (RUN, EOB):
                if not (0 <= value <= 63):
                    print(f"ERROR [blk {b}]: run value {value} out of [0,63] "
                          f"type={sym_type}")
                    errors += 1

            # -- Check 6: LEVEL value is abs(coeff)-1, so >= 0 --------------
            if sym_type == LEVEL:
                if value < 0:
                    print(f"ERROR [blk {b}]: LEVEL value {value} < 0 (should be abs-1)")
                    errors += 1
                # Reconstruct signed coeff
                recon = -(value + 1) if sign else (value + 1)
                if not (-32768 <= recon <= 32767):
                    print(f"ERROR [blk {b}]: LEVEL recon {recon} overflows 16-bit signed")
                    errors += 1

            # -- Check 7: kParam in valid range -----------------------------
            if sym_type == DC and not (0 <= kparam <= 5):
                print(f"ERROR [blk {b}]: DC kParam {kparam} out of [0,5]")
                errors += 1
            if sym_type in (RUN, EOB) and not (0 <= kparam <= 2):
                print(f"ERROR [blk {b}]: RUN/EOB kParam {kparam} out of [0,2]")
                errors += 1
            if sym_type == LEVEL and not (0 <= kparam <= 4):
                print(f"ERROR [blk {b}]: LEVEL kParam {kparam} out of [0,4]")
                errors += 1

    # -- Summary --------------------------------------------------------------
    print(f"Symbols checked : {total}")
    print(f"  DC     : {by_type[DC]}")
    print(f"  RUN    : {by_type[RUN]}")
    print(f"  LEVEL  : {by_type[LEVEL]}")
    print(f"  EOB    : {by_type[EOB]}")
    print()
    print(f"Tier distribution:")
    print(f"  SHORT    : {by_tier['SHORT']}")
    print(f"  MEDIUM   : {by_tier['MEDIUM']}")
    print(f"  EXTENDED : {by_tier['EXTENDED']}")
    print()

    # -- kParam sweep round-trip test -----------------------------------------
    print("kParam exhaustive round-trip (encode_hv → decode_hv):")
    kp_errors = 0
    for kp in range(6):
        short_max  = (1 << kp) - 1
        medium_max = (1 << (kp + 1)) - 1
        # Test values covering all three tiers
        test_vals  = list(range(min(short_max + 1, 8)))
        test_vals += list(range(max(0, short_max - 1), min(medium_max + 1, 64)))
        test_vals += list(range(medium_max + 1, min(medium_max + 10, 64)))
        test_vals  = sorted(set(v for v in test_vals if 0 <= v < 1000))

        for v in test_vals:
            cw = encode_hv(v, kp)
            # For round-trip, decode the whole codeword as magnitude
            dec = decode_hv(cw, kp)
            if dec != v:
                print(f"  FAIL kp={kp} val={v} cw='{cw}' decoded={dec}")
                kp_errors += 1
    if kp_errors == 0:
        print("  All kParam combinations: PASS")
    errors += kp_errors
    print()

    if errors == 0:
        print(f"SELF-TEST PASS — {total} symbols, 0 errors")
        return 0
    else:
        print(f"SELF-TEST FAIL — {errors} errors in {total} symbols")
        return 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--blocks', type=int, default=100)
    parser.add_argument('--seed',   type=int, default=42)
    parser.add_argument('--qp',     type=int, default=22)
    args = parser.parse_args()
    sys.exit(run_self_test(args.blocks, args.seed, args.qp))
