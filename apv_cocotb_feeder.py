"""
apv_cocotb_feeder.py
--------------------
Cocotb 2.x helper: parse an APV file and stream the first tile's
bitstream into your DUT one 32-bit word at a time.

Assumed DUT interface (adjust signal names at the top of this file):
  clk         : input  clock
  rst_n       : input  active-low reset
  data_in     : input  std_logic_vector(31 downto 0)
  data_valid  : input  std_logic
  data_last   : input  std_logic   (asserted on the last word)
  data_ready  : output std_logic   (back-pressure from DUT)

Usage in your test:
  from apv_cocotb_feeder import ApvTileFeeder

  @cocotb.test()
  async def test_tile0(dut):
      feeder = ApvTileFeeder(dut, "path/to/sample.apv", tile=0)
      await feeder.reset()
      await feeder.send_tile()
"""

import os
import cocotb
from cocotb.clock     import Clock
from cocotb.triggers  import RisingEdge, FallingEdge, Timer
from cocotb.handle    import SimHandleBase

# -- Import our APV parser (must be on PYTHONPATH or same directory) --------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from apv_bitstream_feeder import parse_apv_file, bytes_to_words32

# ---------------------------------------------------------------------------
# Tuneable constants
# ---------------------------------------------------------------------------
CLK_PERIOD_NS   = 2        # 500 MHz (match your target)
RESET_CYCLES    = 16       # number of clock cycles held in reset

# DUT signal name mapping (edit these to match your VHDL port names)
SIG_CLK         = "clk"
SIG_RST_N       = "rst_n"
SIG_DATA_IN     = "data_in"        # 32-bit input word
SIG_DATA_VALID  = "data_valid"     # asserted when data_in is valid
SIG_DATA_LAST   = "data_last"      # asserted on last word of tile
SIG_DATA_READY  = "data_ready"     # DUT asserts when it can accept data


# ---------------------------------------------------------------------------
# Helper: resolve a signal by name on the DUT handle
# ---------------------------------------------------------------------------

def _sig(dut: SimHandleBase, name: str):
    try:
        return getattr(dut, name)
    except AttributeError:
        raise AttributeError(
            f"DUT has no signal '{name}'. "
            f"Edit the SIG_* constants in apv_cocotb_feeder.py to match "
            f"your VHDL port names."
        )


# ---------------------------------------------------------------------------
# ApvTileFeeder class
# ---------------------------------------------------------------------------

class ApvTileFeeder:
    """
    Parse an APV file and stream one tile as 32-bit words into a DUT.

    Parameters
    ----------
    dut         : cocotb DUT handle
    apv_path    : path to .apv file
    tile        : 0-based tile index (default 0 = first tile)
    comp        : if not None, send only this component's entropy data;
                  if None (default), send concatenated entropy data for
                  all components with the tile_header stripped.
    back_pressure : if True, honour dut.data_ready for flow control
    """

    def __init__(
        self,
        dut,
        apv_path: str,
        tile: int = 0,
        comp: int | None = None,
        back_pressure: bool = True,
    ):
        self.dut          = dut
        self.apv_path     = apv_path
        self.tile_idx     = tile
        self.comp         = comp
        self.back_pressure = back_pressure

        # Parse once at construction time
        result = parse_apv_file(apv_path, target_tile=tile)
        self.frame_info  = result['frame_info']
        self.tile_header = result['tile_header']

        if comp is not None:
            raw = result['tile_comp_data'][comp]
        else:
            raw = b''.join(result['tile_comp_data'])

        self.words = bytes_to_words32(raw, pad=True)

        cocotb.log.info(
            f"[ApvTileFeeder] Loaded tile {tile} from {os.path.basename(apv_path)}: "
            f"{len(raw)} bytes → {len(self.words)} words @ 32b"
        )
        cocotb.log.info(
            f"[ApvTileFeeder] Frame: {self.frame_info['comp_info'][0]['width']}x"
            f"{self.frame_info['comp_info'][0]['height']} "
            f"{self.frame_info['bit_depth']}-bit  "
            f"tiles={self.frame_info['num_tiles']}"
        )

    # ------------------------------------------------------------------
    # Clock start helper (call once per test if you don't start it externally)
    # ------------------------------------------------------------------

    def start_clock(self):
        cocotb.start_soon(
            Clock(
                _sig(self.dut, SIG_CLK),
                CLK_PERIOD_NS,
                units="ns"
            ).start()
        )

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    async def reset(self, start_clock: bool = True):
        """Drive reset low for RESET_CYCLES, then release."""
        if start_clock:
            self.start_clock()

        clk   = _sig(self.dut, SIG_CLK)
        rst_n = _sig(self.dut, SIG_RST_N)

        # De-assert handshake signals
        _sig(self.dut, SIG_DATA_VALID).value = 0
        _sig(self.dut, SIG_DATA_LAST).value  = 0
        _sig(self.dut, SIG_DATA_IN).value    = 0

        rst_n.value = 0
        for _ in range(RESET_CYCLES):
            await RisingEdge(clk)
        rst_n.value = 1
        await RisingEdge(clk)

        cocotb.log.info("[ApvTileFeeder] Reset complete")

    # ------------------------------------------------------------------
    # Send the tile
    # ------------------------------------------------------------------

    async def send_tile(self):
        """
        Stream all words into the DUT one per clock cycle.
        Handles back-pressure via data_ready if back_pressure=True.
        """
        clk        = _sig(self.dut, SIG_CLK)
        data_in    = _sig(self.dut, SIG_DATA_IN)
        data_valid = _sig(self.dut, SIG_DATA_VALID)
        data_last  = _sig(self.dut, SIG_DATA_LAST)

        total = len(self.words)
        cocotb.log.info(
            f"[ApvTileFeeder] Sending tile {self.tile_idx}: "
            f"{total} words"
        )

        for i, word in enumerate(self.words):
            is_last = (i == total - 1)

            # Wait for DUT to be ready (back-pressure)
            if self.back_pressure:
                while True:
                    await RisingEdge(clk)
                    try:
                        rdy = _sig(self.dut, SIG_DATA_READY)
                        if int(rdy.value) == 1:
                            break
                    except AttributeError:
                        # No data_ready port — proceed without back-pressure
                        self.back_pressure = False
                        break
            else:
                await RisingEdge(clk)

            data_in.value    = word
            data_valid.value = 1
            data_last.value  = 1 if is_last else 0

            if i % 64 == 0:
                cocotb.log.debug(
                    f"[ApvTileFeeder] word[{i:4d}/{total}] = 0x{word:08X}"
                )

        # De-assert valid after last word
        await RisingEdge(clk)
        data_valid.value = 0
        data_last.value  = 0
        data_in.value    = 0

        cocotb.log.info(
            f"[ApvTileFeeder] Tile {self.tile_idx} transmission complete"
        )

    # ------------------------------------------------------------------
    # Convenience: reset then send
    # ------------------------------------------------------------------

    async def run(self):
        await self.reset()
        await self.send_tile()

    # ------------------------------------------------------------------
    # Inspection helpers (call from test assertions)
    # ------------------------------------------------------------------

    def get_words(self) -> list[int]:
        """Return the full list of 32-bit words (for golden reference)."""
        return list(self.words)

    def get_word(self, index: int) -> int:
        return self.words[index]

    def word_count(self) -> int:
        return len(self.words)

    def dump_hex(self, max_words: int = 32) -> str:
        """Return a compact hex dump for logging."""
        lines = []
        for i in range(min(max_words, len(self.words))):
            lines.append(f"  [{i:4d}] 0x{self.words[i]:08X}")
        if len(self.words) > max_words:
            lines.append(f"  ... ({len(self.words) - max_words} more words)")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Minimal standalone test (run with: cocotb + your DUT)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Standalone sanity check without DUT — just parse and print
    import argparse
    from apv_bitstream_feeder import parse_apv_file, bytes_to_words32

    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--tile", type=int, default=0)
    ap.add_argument("--comp", type=int, default=None)
    args = ap.parse_args()

    result = parse_apv_file(args.file, target_tile=args.tile)
    fi = result['frame_info']
    th = result['tile_header']

    if args.comp is not None:
        raw = result['tile_comp_data'][args.comp]
    else:
        raw = b''.join(result['tile_comp_data'])

    words = bytes_to_words32(raw, pad=True)

    print(f"Tile {args.tile}: {len(raw)} bytes → {len(words)} words")
    print(f"First 8 words:")
    for i, w in enumerate(words[:8]):
        print(f"  [{i}] 0x{w:08X}  {w:032b}")
