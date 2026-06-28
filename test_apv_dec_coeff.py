"""
test_apv_dec_coeff.py
=====================
cocotb testbench for apv_dec_coeff.vhd  --  APV h(v) VLC coefficient decoder.
"""

import cocotb
from cocotb.clock    import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ClockCycles
import random
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from apv_encode_pipeline import (
    encode_block, encode_hv, fdct_2d, quantise, zigzag_scan,
    DC, RUN, LEVEL, EOB,
    special_cases,
)

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
CLK_PERIOD_NS = 2        # 500 MHz
DATA_W        = 34
BIT_DEPTH     = 10
BACK_TO_BACK  = False    # set True for continuous back-to-back drive

# QP: fixed from environment variable, or None -> random per block
_qp_env = os.environ.get('APV_QP')
FIXED_QP = int(_qp_env) if _qp_env is not None else None


# -----------------------------------------------------------------------------
# Golden symbol generator
# -----------------------------------------------------------------------------
def generate_golden_symbols(num_blocks: int = 20, seed: int = 42):
    """
    Encodes num_blocks random 8x8 blocks followed by special_cases() blocks.

    Each returned dict contains:
      'type'     : DC | RUN | LEVEL | EOB
      'codeword' : binary string (magnitude bits + sign bit for DC/LEVEL)
      'block'    : block index
      'exp_coeff': expected signed coeff 
      'exp_run'  : expected run value
    """
    rng = random.Random(seed)
    MIN10, MAX10 = -(1 << (BIT_DEPTH - 1)), (1 << (BIT_DEPTH - 1)) - 1

    random_rows = [[rng.randint(MIN10, MAX10) for _ in range(8)]
                   for _ in range(num_blocks * 8)]
    all_rows   = random_rows + special_cases()
    num_blocks = len(all_rows) // 8

    all_symbols = []
    prev_dc = 0
    kp_dc = kp_run = kp_lvl = 0

    for b in range(num_blocks):
        qp    = FIXED_QP if FIXED_QP is not None else rng.randint(0, 51)
        block = all_rows[b*8 : b*8+8]
        zz    = zigzag_scan(quantise(fdct_2d(block, BIT_DEPTH), qp, BIT_DEPTH))

        symbols, prev_dc, kp_dc, kp_run, kp_lvl = encode_block(
            zz, prev_dc, kp_dc, kp_run, kp_lvl)
        kp_run = 0
        kp_lvl = 0

        for sym_type, value, sign, kparam, codeword in symbols:
            rec = {
                'type'     : sym_type,
                'codeword' : codeword,
                'block'    : b,
                'qp'       : qp,
                'exp_coeff': None,
                'exp_run'  : None,
            }
            if sym_type == DC:
                rec['exp_coeff'] = -value if sign else value
            elif sym_type == LEVEL:
                rec['exp_coeff'] = -(value + 1) if sign else (value + 1)
            elif sym_type in (RUN, EOB):
                rec['exp_run'] = value

            all_symbols.append(rec)

    return all_symbols


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
def codeword_to_window(codeword: str) -> int:
    """Left-justify codeword into DATA_W bits, zero-pad on the right."""
    return int(codeword.ljust(DATA_W, '0')[:DATA_W], 2)


async def reset_dut(dut, cycles: int = 8):
    dut.rst_n.value   = 0
    dut.i_raw_bits_valid.value = 0
    dut.i_raw_bits.value  = 0
    await ClockCycles(dut.clk, cycles)
    dut.rst_n.value   = 1
    await RisingEdge(dut.clk)


# -----------------------------------------------------------------------------
# Core drive + check
# -----------------------------------------------------------------------------
async def drive_and_check(dut, sym: dict, sym_idx: int) -> bool:
    """
    Drive one 34-bit codeword window and check all DUT outputs.
    """
    type_name = {DC: 'DC', RUN: 'RUN', LEVEL: 'LVL', EOB: 'EOB'}[sym['type']]
    tag = (f"[#{sym_idx} blk={sym['block']} qp={sym['qp']} "
           f"{type_name} cw='{sym['codeword']}']")

    pass_flag = True
    exp_shift = len(sym['codeword'])

    # -- Cycle 1: drive input, sample o_shift ---------------------------------
    dut.i_raw_bits.value  = codeword_to_window(sym['codeword'])
    dut.i_raw_bits_valid.value = 1
    await RisingEdge(dut.clk)
    
    # -- Sample o_shift -------------------------------------------------------
    await ReadOnly()
    rcv_shift = int(dut.o_shift.value)
    if rcv_shift != exp_shift:
        dut._log.error(f"SHIFT FAIL {tag}: exp={exp_shift} rcv={rcv_shift}")
        pass_flag = False
    else:
        dut._log.debug(f"SHIFT OK {tag}: {rcv_shift}")
    if not BACK_TO_BACK:
        dut.i_raw_bits_valid.value = 0
        dut.i_raw_bits.value  = 0


    # -- Cycle 2: sample o_coeff / o_run --------------------------------------
    await RisingEdge(dut.clk)
    await ReadOnly()

    if sym['type'] == DC:
        rcv = dut.o_coeff.value.signed_integer
        if rcv != sym['exp_coeff']:
            dut._log.error(f"DC_COEFF FAIL {tag}: exp={sym['exp_coeff']} rcv={rcv}")
            pass_flag = False
        else:
            dut._log.debug(f"DC_COEFF OK {tag}: {rcv}")

    elif sym['type'] == LEVEL:
        rcv = dut.o_coeff.value.signed_integer
        if rcv != sym['exp_coeff']:
            dut._log.error(f"AC_COEFF FAIL {tag}: exp={sym['exp_coeff']} rcv={rcv}")
            pass_flag = False
        else:
            dut._log.debug(f"AC_COEFF OK {tag}: {rcv}")

    elif sym['type'] in (RUN, EOB):
        rcv = int(dut.o_run.value)
        if rcv != sym['exp_run']:
            dut._log.error(f"RUN FAIL {tag}: exp={sym['exp_run']} rcv={rcv}")
            pass_flag = False
        else:
            dut._log.debug(f"RUN OK {tag}: {rcv}")

    # -- Optional idle gap -----------------------------------------------------
    if not BACK_TO_BACK:
        await RisingEdge(dut.clk)

    return pass_flag


# =============================================================================
# TEST 1 -- Full stream
# =============================================================================
@cocotb.test()
async def test_full_stream(dut):
    """
    Main regression: all symbol types in encoder order.
    Checks o_shift (cycle 1) and o_coeff/o_run (cycle 2) for every symbol.
    QP is random per block unless APV_QP is set.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units='ns').start())
    await reset_dut(dut)

    golden = generate_golden_symbols(num_blocks=50, seed=42)
    qp_mode = f"fixed QP={FIXED_QP}" if FIXED_QP is not None else "random QP per block"
    dut._log.info(f"test_full_stream: {len(golden)} symbols  {qp_mode}  "
                  f"({'back-to-back' if BACK_TO_BACK else 'with idle gaps'})")

    errors = 0
    for idx, sym in enumerate(golden):
        if not await drive_and_check(dut, sym, idx):
            errors += 1

    if BACK_TO_BACK:
        dut.i_raw_bits_valid.value = 0

    assert errors == 0, f"{errors}/{len(golden)} failures"
    dut._log.info("test_full_stream PASS")


# =============================================================================
# TEST 2 -- kParam x tier shift sweep
# =============================================================================
@cocotb.test()
async def test_kparam_shift_sweep(dut):
    """
    Exhaustive: for each kParam (0..5) drive SHORT / MEDIUM / EXTENDED
    codewords and verify o_shift == len(codeword).
    White-box test targeting the shift computation path.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units='ns').start())
    await reset_dut(dut)

    errors  = 0
    checked = 0

    for kp in range(6):
        short_max  = (1 << kp) - 1
        medium_max = (1 << (kp + 1)) - 1

        test_values  = list(range(min(short_max + 1, 4)))
        test_values += [short_max] if short_max > 0 else []
        test_values += [short_max + 1, medium_max] if medium_max > short_max else []
        for extra in (medium_max + 1, medium_max + 4, medium_max + (1 << kp)):
            if 0 < extra < 64:
                test_values.append(extra)

        for val in sorted(set(test_values)):
            cw  = encode_hv(val, kp)
            exp = len(cw)

            dut.i_raw_bits.value  = codeword_to_window(cw)
            dut.i_raw_bits_valid.value = 1
            await RisingEdge(dut.clk)   # cycle 1

            if not BACK_TO_BACK:
                dut.i_raw_bits_valid.value = 0
                dut.i_raw_bits.value  = 0

            await ReadOnly()

            rcv = int(dut.o_shift.value)
            if rcv != exp:
                dut._log.error(f"kp={kp} val={val} cw='{cw}' exp={exp} rcv={rcv}")
                errors += 1
            checked += 1

            # consume cycle 2 and optional idle before next symbol
            await RisingEdge(dut.clk)
            if not BACK_TO_BACK:
                await RisingEdge(dut.clk)

    if BACK_TO_BACK:
        dut.i_raw_bits_valid.value = 0

    assert errors == 0, f"{errors} shift failures"
    dut._log.info(f"test_kparam_shift_sweep PASS  {checked} combinations")


# =============================================================================
# TEST 3 -- Corner cases
# =============================================================================
@cocotb.test()
async def test_corner_cases(dut):
    """
    special_cases() blocks: all-max, all-min, alternating, single-spike.
    Exercises EXTENDED tier Exp-Golomb codewords and run=0 interleaving.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units='ns').start())
    await reset_dut(dut)

    golden = generate_golden_symbols(num_blocks=0, seed=0)
    dut._log.info(f"test_corner_cases: {len(golden)} symbols")

    errors = 0
    for idx, sym in enumerate(golden):
        if not await drive_and_check(dut, sym, idx):
            errors += 1

    if BACK_TO_BACK:
        dut.i_raw_bits_valid.value = 0

    assert errors == 0, f"{errors}/{len(golden)} failures"
    dut._log.info("test_corner_cases PASS")


# =============================================================================
# TEST 4 -- Ghost output check (i_raw_bits_valid gating)
# =============================================================================
@cocotb.test()
async def test_valid_gating(dut):
    """
    After each symbol, hold i_raw_bits_valid low for 3 cycles and verify that
    o_dc_valid / o_ac_valid / o_run_valid all remain 0.
    Always runs in non-back-to-back mode.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units='ns').start())
    await reset_dut(dut)

    golden = generate_golden_symbols(num_blocks=5, seed=7)
    errors = 0

    for idx, sym in enumerate(golden[:20]):
        # drive for one cycle
        dut.i_raw_bits.value  = codeword_to_window(sym['codeword'])
        dut.i_raw_bits_valid.value = 1
        await RisingEdge(dut.clk)
        dut.i_raw_bits_valid.value = 0
        dut.i_raw_bits.value  = 0

        # 3 idle cycles starting from cycle 2 onward -- no valid must fire
        for idle in range(3):
            await RisingEdge(dut.clk)
            await ReadOnly()
            dc_v  = int(dut.o_dc_valid.value)
            ac_v  = int(dut.o_ac_valid.value)
            run_v = int(dut.o_run_valid.value)
            if dc_v or ac_v or run_v:
                dut._log.error(
                    f"[#{idx}] GHOST OUTPUT idle+{idle+1}: "
                    f"dc={dc_v} ac={ac_v} run={run_v}")
                errors += 1

    assert errors == 0, f"{errors} ghost output(s)"
    dut._log.info("test_valid_gating PASS")


# =============================================================================
# TEST 5 -- Smooth (image-like) data
# =============================================================================
@cocotb.test()
async def test_smooth_data(dut):
    """
    Spatially smooth blocks: long zero runs, small DC diffs, few AC levels.
    Stresses the RUN path and SHORT tier codewords.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units='ns').start())
    await reset_dut(dut)

    rng = random.Random(55)
    MIN10, MAX10 = -(1 << (BIT_DEPTH - 1)), (1 << (BIT_DEPTH - 1)) - 1
    prev_dc = 0
    kp_dc = kp_run = kp_lvl = 0
    all_symbols = []

    block_base = rng.randint(MIN10 // 2, MAX10 // 2)
    for b in range(40):
        if b % 8 == 0:
            block_base = max(MIN10, min(MAX10, block_base + rng.randint(-20, 20)))
        qp    = FIXED_QP if FIXED_QP is not None else rng.randint(0, 51)
        block = [[max(MIN10, min(MAX10, block_base + rng.randint(-8, 8)))
                  for _ in range(8)] for _ in range(8)]
        zz = zigzag_scan(quantise(fdct_2d(block, BIT_DEPTH), qp, BIT_DEPTH))
        syms, prev_dc, kp_dc, kp_run, kp_lvl = encode_block(
            zz, prev_dc, kp_dc, kp_run, kp_lvl)
        kp_run = 0
        kp_lvl = 0

        for sym_type, value, sign, kparam, codeword in syms:
            rec = {'type': sym_type, 'codeword': codeword, 'block': b, 'qp': qp,
                   'exp_coeff': None, 'exp_run': None}
            if sym_type == DC:
                rec['exp_coeff'] = -value if sign else value
            elif sym_type == LEVEL:
                rec['exp_coeff'] = -(value + 1) if sign else (value + 1)
            elif sym_type in (RUN, EOB):
                rec['exp_run'] = value
            all_symbols.append(rec)

    dut._log.info(f"test_smooth_data: {len(all_symbols)} symbols")
    errors = 0
    for idx, sym in enumerate(all_symbols):
        if not await drive_and_check(dut, sym, idx):
            errors += 1

    if BACK_TO_BACK:
        dut.i_raw_bits_valid.value = 0

    assert errors == 0, f"{errors}/{len(all_symbols)} failures"
    dut._log.info("test_smooth_data PASS")


# =============================================================================
# TEST 6 -- Reset recovery
# =============================================================================
@cocotb.test()
async def test_reset_recovery(dut):
    """
    Send 10 symbols, assert mid-stream reset, send 10 more, verify both halves.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units='ns').start())
    await reset_dut(dut)

    golden = generate_golden_symbols(num_blocks=10, seed=13)
    errors = 0

    for idx, sym in enumerate(golden[:10]):
        if not await drive_and_check(dut, sym, idx):
            errors += 1

    dut._log.info("Applying mid-stream reset...")
    dut.i_raw_bits_valid.value = 0
    dut.i_raw_bits.value  = 0
    await reset_dut(dut, cycles=4)

    for idx, sym in enumerate(golden[10:20]):
        if not await drive_and_check(dut, sym, idx + 10):
            errors += 1

    if BACK_TO_BACK:
        dut.i_raw_bits_valid.value = 0

    assert errors == 0, f"{errors} failures"
    dut._log.info("test_reset_recovery PASS")
