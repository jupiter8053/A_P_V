#!/usr/bin/env python3
"""
apv_bitstream_feeder.py
-----------------------
Parse a raw APV (.apv) file (RFC 9924 / Appendix A raw bitstream format),
extract a specific tile's entropy-coded component data, and emit it as
32-bit big-endian words for FPGA simulation.

Raw .apv file layout (RFC 9924 Appendix A):
  [au_size        : u32]   size in bytes of the access_unit() that follows
  access_unit():
    [signature     : f32]  0x61507631 ('aPv1')
    do {
      [pbu_size    : u32]  size in bytes of the pbu() that follows
      pbu():
        pbu_header():
          [pbu_type          : u8 ]
          [group_id          : u16]
          [reserved_zero_8bits: u8 ]
        frame():                      <- when pbu_type in {1,2,25,26,27}
          frame_header():
            frame_info():
              [profile_idc           : u8 ]
              [level_idc             : u8 ]
              [band_idc              : u3 ]
              [reserved_zero_5bits   : u5 ]
              [frame_width           : u24]   <- luma pixels
              [frame_height          : u24]   <- luma pixels
              [chroma_format_idc     : u4 ]
              [bit_depth_minus8      : u4 ]
              [capture_time_distance : u8 ]
              [reserved_zero_8bits   : u8 ]
            [reserved_zero_8bits     : u8 ]
            [color_description_present_flag: u1]
            if color_description_present_flag:
              [color_primaries        : u8]
              [transfer_characteristics: u8]
              [matrix_coefficients    : u8]
              [full_range_flag        : u1]
            [use_q_matrix            : u1]
            if use_q_matrix:
              quantization_matrix()   <- NumComps * 64 bytes
            tile_info():
              [tile_width_in_mbs     : u20]
              [tile_height_in_mbs    : u20]
              ... derives NumTiles ...
              [tile_size_present_in_fh_flag: u1]
              if tile_size_present_in_fh_flag:
                [tile_size_in_fh[i]  : u32] * NumTiles
            [reserved_zero_8bits     : u8 ]
            byte_alignment()
          for i in 0..NumTiles-1:
            [tile_size[i]            : u32]   <- bytes of tile(i)
            tile(i):
              tile_header(i):
                [tile_header_size    : u16]   <- size of tile_header in bytes
                [tile_index          : u16]
                for c in 0..NumComps-1:
                  [tile_data_size[c] : u32]
                for c in 0..NumComps-1:
                  [tile_qp[c]        : u8 ]
                [reserved_zero_8bits : u8 ]
                byte_alignment()
              for c in 0..NumComps-1:
                tile_data(i,c)        <- tile_data_size[c] bytes of VLC data

NumComps is derived from chroma_format_idc (RFC 9924 Table 2):
  0 -> 1  (4:0:0)
  2 -> 3  (4:2:2)
  3 -> 3  (4:4:4)
  4 -> 4  (4:4:4:4)

Usage:
  python apv_bitstream_feeder.py sample.apv
  python apv_bitstream_feeder.py sample.apv --tile 0 --comp 0
  python apv_bitstream_feeder.py sample.apv --cocotb > tile0_vectors.py
"""

import struct
import sys
import math
import argparse

# ---------------------------------------------------------------------------
# chroma_format_idc -> NumComps  (RFC 9924 Table 2)
# ---------------------------------------------------------------------------
CHROMA_FMT_TO_NUM_COMPS = {0: 1, 2: 3, 3: 3, 4: 4}


# ---------------------------------------------------------------------------
# Bit reader
# ---------------------------------------------------------------------------

class BitReader:
    """Read individual bits and multi-bit fields from a bytes buffer."""

    def __init__(self, data: bytes, start_byte: int = 0):
        self._data     = data
        self._byte_pos = start_byte
        self._bit_pos  = 0   # 0 = MSB of current byte

    # ---- position helpers ------------------------------------------------

    @property
    def byte_pos(self) -> int:
        """Current byte position (next whole byte boundary >= consumed bits)."""
        return self._byte_pos + (1 if self._bit_pos > 0 else 0)

    @property
    def bit_offset(self) -> int:
        """Total bits consumed so far."""
        return self._byte_pos * 8 + self._bit_pos

    def is_byte_aligned(self) -> bool:
        return self._bit_pos == 0

    # ---- core read -------------------------------------------------------

    def read_bits(self, n: int) -> int:
        """Read n bits MSB-first and return as unsigned integer."""
        if n == 0:
            return 0
        avail = (len(self._data) - self._byte_pos) * 8 - self._bit_pos
        if avail < n:
            raise EOFError(
                f"Need {n} bits but only {avail} available "
                f"(byte={self._byte_pos}, bit={self._bit_pos})"
            )
        result    = 0
        remaining = n
        while remaining > 0:
            bits_in_byte = 8 - self._bit_pos
            take         = min(bits_in_byte, remaining)
            shift        = bits_in_byte - take
            mask         = (1 << take) - 1
            bits         = (self._data[self._byte_pos] >> shift) & mask
            result       = (result << take) | bits
            self._bit_pos += take
            remaining    -= take
            if self._bit_pos == 8:
                self._byte_pos += 1
                self._bit_pos   = 0
        return result

    # ---- convenience wrappers --------------------------------------------

    def u(self, n: int) -> int:
        return self.read_bits(n)

    def u8(self)  -> int: return self.read_bits(8)
    def u16(self) -> int: return self.read_bits(16)
    def u32(self) -> int: return self.read_bits(32)
    def u24(self) -> int: return self.read_bits(24)

    # ---- alignment -------------------------------------------------------

    def byte_align(self):
        """Consume padding bits until the reader is byte-aligned."""
        if self._bit_pos != 0:
            self._byte_pos += 1
            self._bit_pos   = 0

    def skip_bytes(self, n: int):
        """Skip n bytes (must be called when byte-aligned)."""
        self.byte_align()
        self._byte_pos += n

    def read_bytes(self, n: int) -> bytes:
        """Read n raw bytes (must be called when byte-aligned)."""
        self.byte_align()
        start           = self._byte_pos
        self._byte_pos += n
        return self._data[start : self._byte_pos]

    def peek_bytes(self, n: int) -> bytes:
        """Peek at the next n bytes without advancing."""
        self.byte_align()
        return self._data[self._byte_pos : self._byte_pos + n]


# ---------------------------------------------------------------------------
# frame_info()  -- RFC 9924 Section 5.3.6
# ---------------------------------------------------------------------------

def parse_frame_info(br: BitReader) -> dict:
    fi = {}
    fi['profile_idc']           = br.u8()
    fi['level_idc']             = br.u8()
    fi['band_idc']              = br.u(3)
    fi['reserved_zero_5bits']   = br.u(5)
    fi['frame_width']           = br.u24()   # luma pixels
    fi['frame_height']          = br.u24()   # luma pixels
    fi['chroma_format_idc']     = br.u(4)
    fi['bit_depth_minus8']      = br.u(4)
    fi['capture_time_distance'] = br.u8()
    fi['reserved_zero_8bits']   = br.u8()

    fi['BitDepth']  = fi['bit_depth_minus8'] + 8
    fi['NumComps']  = CHROMA_FMT_TO_NUM_COMPS.get(fi['chroma_format_idc'], 3)

    # Derived frame geometry (RFC 9924 Section 5.3.6)
    MbWidth  = 16
    MbHeight = 16
    fi['FrameWidthInMbsY']  = math.ceil(fi['frame_width']  / MbWidth)
    fi['FrameHeightInMbsY'] = math.ceil(fi['frame_height'] / MbHeight)

    return fi


# ---------------------------------------------------------------------------
# quantization_matrix()  -- RFC 9924 Section 5.3.7
# ---------------------------------------------------------------------------

def skip_quantization_matrix(br: BitReader, num_comps: int):
    """Each component has an 8x8 matrix of u8 entries = 64 bytes = 512 bits.
    These are read as contiguous u8 bitfield entries with NO byte-alignment
    padding before them (they follow directly after the use_q_matrix flag bit
    inside the same bitfield).  Must use read_bits, not skip_bytes."""
    br.read_bits(num_comps * 64 * 8)


# ---------------------------------------------------------------------------
# tile_info()  -- RFC 9924 Section 5.3.8
# ---------------------------------------------------------------------------

def parse_tile_info(br: BitReader, fi: dict) -> dict:
    ti = {}
    ti['tile_width_in_mbs']  = br.u(20)
    ti['tile_height_in_mbs'] = br.u(20)

    # Derive tile grid (same loop logic as spec pseudocode)
    FW = fi['FrameWidthInMbsY']
    FH = fi['FrameHeightInMbsY']
    tw = ti['tile_width_in_mbs']
    th = ti['tile_height_in_mbs']

    tile_cols = math.ceil(FW / tw)
    tile_rows = math.ceil(FH / th)

    ti['TileCols'] = tile_cols
    ti['TileRows'] = tile_rows
    ti['NumTiles'] = tile_cols * tile_rows

    # Optional per-frame tile_size array
    ti['tile_size_present_in_fh_flag'] = br.u(1)
    ti['tile_size_in_fh'] = []
    if ti['tile_size_present_in_fh_flag']:
        for _ in range(ti['NumTiles']):
            ti['tile_size_in_fh'].append(br.u32())

    return ti


# ---------------------------------------------------------------------------
# frame_header()  -- RFC 9924 Section 5.3.5
# ---------------------------------------------------------------------------

def parse_frame_header(br: BitReader) -> tuple[dict, dict]:
    """
    Returns (frame_info dict, tile_info dict).
    Leaves br positioned at the first tile_size field.
    """
    fi = parse_frame_info(br)

    # Fields that follow frame_info() inside frame_header()
    br.u8()                                        # reserved_zero_8bits
    color_flag = br.u(1)
    fi['color_description_present_flag'] = color_flag
    fi['color_primaries']         = 0
    fi['transfer_characteristics']= 0
    fi['matrix_coefficients']     = 0
    fi['full_range_flag']         = 0
    if color_flag:
        fi['color_primaries']          = br.u8()
        fi['transfer_characteristics'] = br.u8()
        fi['matrix_coefficients']      = br.u8()
        fi['full_range_flag']          = br.u(1)

    use_q = br.u(1)
    fi['use_q_matrix'] = use_q
    if use_q:
        skip_quantization_matrix(br, fi['NumComps'])

    ti = parse_tile_info(br, fi)

    br.u8()          # reserved_zero_8bits (end of frame_header)
    br.byte_align()  # byte_alignment()

    return fi, ti


# ---------------------------------------------------------------------------
# tile_header()  -- RFC 9924 Section 5.3.13
# ---------------------------------------------------------------------------

def parse_tile_header(br: BitReader, num_comps: int) -> dict:
    """
    tile_header():
      tile_header_size  u16
      tile_index        u16
      tile_data_size[i] u32  x NumComps
      tile_qp[i]        u8   x NumComps
      reserved_zero_8bits u8
      byte_alignment()
    """
    th = {}
    th['tile_header_size'] = br.u16()   # size of tile_header in bytes
    th['tile_index']       = br.u16()
    th['tile_data_size']   = [br.u32() for _ in range(num_comps)]
    th['tile_qp']          = [br.u8()  for _ in range(num_comps)]
    br.u8()          # reserved_zero_8bits
    br.byte_align()  # byte_alignment()
    return th


# ---------------------------------------------------------------------------
# Main parse entry point
# ---------------------------------------------------------------------------

def parse_apv_file(path: str, target_tile: int = 0) -> dict:
    """
    Parse a raw .apv file (RFC 9924 Appendix A format) and return info
    about the requested tile.

    Returns dict:
      frame_info       : dict  (from frame_header)
      tile_info        : dict  (from tile_info)
      tile_header      : dict  (from tile_header of target tile)
      tile_comp_data   : list[bytes]  per-component entropy-coded bytes
      tile_size_field  : int   value of tile_size[i] field in frame()
      offsets          : dict  byte offsets of key structures in the file
    """
    with open(path, 'rb') as f:
        raw = f.read()

    if len(raw) < 8:
        raise ValueError("File too short to be a valid raw APV bitstream")

    br = BitReader(raw)

    # ------------------------------------------------------------------
    # Raw bitstream wrapper  (Appendix A)
    # ------------------------------------------------------------------
    au_size   = br.u32()   # size of access_unit() that follows
    offsets   = {'au_size': 0, 'access_unit': 4}

    # ------------------------------------------------------------------
    # access_unit():  signature + one or more PBUs
    # ------------------------------------------------------------------
    signature = br.u32()
    if signature != 0x61507631:
        raise ValueError(
            f"Bad APV signature: 0x{signature:08X} "
            f"(expected 0x61507631 'aPv1')"
        )
    offsets['signature'] = 4

    # ------------------------------------------------------------------
    # First PBU  (we look for the primary frame PBU, pbu_type == 1)
    # ------------------------------------------------------------------
    pbu_size   = br.u32()
    offsets['pbu_size']   = 8
    offsets['pbu_header'] = 12

    pbu_type   = br.u8()
    group_id   = br.u16()
    _reserved  = br.u8()

    if pbu_type not in (1, 2, 25, 26, 27):
        raise ValueError(
            f"First PBU has pbu_type={pbu_type}, expected a frame PBU "
            f"(1=primary, 2=non-primary, 25=preview, 26=depth, 27=alpha)"
        )

    offsets['frame_header'] = br._byte_pos

    # ------------------------------------------------------------------
    # frame_header()
    # ------------------------------------------------------------------
    fi, ti = parse_frame_header(br)

    num_comps = fi['NumComps']
    num_tiles = ti['NumTiles']

    if target_tile >= num_tiles:
        raise ValueError(
            f"Requested tile {target_tile} but frame has only "
            f"{num_tiles} tiles ({ti['TileCols']}x{ti['TileRows']})"
        )

    offsets['first_tile'] = br._byte_pos

    # ------------------------------------------------------------------
    # Walk tile_size[i] + tile(i) to reach the target tile
    # ------------------------------------------------------------------
    for tile_idx in range(num_tiles):
        tile_size_offset = br._byte_pos
        tile_size_val    = br.u32()

        tile_data_start  = br._byte_pos   # start of tile() payload
        tile_data_end    = tile_data_start + tile_size_val

        if tile_data_end > len(raw):
            raise ValueError(
                f"Tile {tile_idx}: tile_size={tile_size_val} extends "
                f"past EOF (file size={len(raw)})"
            )

        if tile_idx == target_tile:
            offsets['tile_size_field'] = tile_size_offset
            offsets['tile_data_start'] = tile_data_start

            # Parse tile_header
            th = parse_tile_header(br, num_comps)

            # Extract per-component entropy-coded data
            comp_data = []
            for c in range(num_comps):
                sz = th['tile_data_size'][c]
                comp_data.append(br.read_bytes(sz))

            return {
                'frame_info':      fi,
                'tile_info':       ti,
                'tile_header':     th,
                'tile_comp_data':  comp_data,
                'tile_size_field': tile_size_val,
                'offsets':         offsets,
                'pbu_type':        pbu_type,
                'group_id':        group_id,
                'au_size':         au_size,
            }

        # Skip to next tile
        br._byte_pos = tile_data_end
        br._bit_pos  = 0

    raise RuntimeError(f"Tile {target_tile} not found after walking {num_tiles} tiles")


# ---------------------------------------------------------------------------
# 32-bit word packer
# ---------------------------------------------------------------------------

def bytes_to_words32(data: bytes, pad: bool = True) -> list[int]:
    """Pack bytes into 32-bit big-endian words, zero-padding the last word."""
    if pad:
        rem = len(data) % 4
        if rem:
            data = data + b'\x00' * (4 - rem)
    return [struct.unpack_from('>I', data, i)[0] for i in range(0, len(data), 4)]


# ---------------------------------------------------------------------------
# Cocotb vector string
# ---------------------------------------------------------------------------

def cocotb_vector_string(words: list[int], label: str = "TILE_WORDS") -> str:
    lines = [f"{label} = ["]
    for i, w in enumerate(words):
        sep = "," if i < len(words) - 1 else ""
        lines.append(f"    0x{w:08X}{sep}  # [{i}]")
    lines.append("]")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Pretty print
# ---------------------------------------------------------------------------

def print_frame_info(fi: dict):
    print("=== frame_info ===")
    print(f"  profile_idc        : {fi['profile_idc']}")
    print(f"  level_idc          : {fi['level_idc']}")
    print(f"  band_idc           : {fi['band_idc']}")
    print(f"  frame_width        : {fi['frame_width']} px")
    print(f"  frame_height       : {fi['frame_height']} px")
    print(f"  chroma_format_idc  : {fi['chroma_format_idc']}")
    print(f"  bit_depth          : {fi['BitDepth']}")
    print(f"  NumComps           : {fi['NumComps']}")
    print(f"  use_q_matrix       : {fi['use_q_matrix']}")


def print_tile_info(ti: dict):
    print("=== tile_info ===")
    print(f"  tile_width_in_mbs  : {ti['tile_width_in_mbs']}")
    print(f"  tile_height_in_mbs : {ti['tile_height_in_mbs']}")
    print(f"  TileCols x TileRows: {ti['TileCols']} x {ti['TileRows']}"
          f" = {ti['NumTiles']} tiles")


def print_tile_header(th: dict):
    print("=== tile_header ===")
    print(f"  tile_header_size   : {th['tile_header_size']} bytes")
    print(f"  tile_index         : {th['tile_index']}")
    for i, (sz, qp) in enumerate(
        zip(th['tile_data_size'], th['tile_qp'])
    ):
        print(f"  comp[{i}]  data_size={sz} bytes  qp={qp}")


def print_words(words: list[int], label: str, max_words: int = 256):
    n = min(len(words), max_words)
    print(f"\n=== {label} ({len(words)} x 32-bit words) ===")
    for i in range(n):
        print(f"  [{i:4d}] 0x{words[i]:08X}  {words[i]:032b}")
    if len(words) > max_words:
        print(f"  ... ({len(words) - max_words} more words not shown)")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Parse raw APV file and emit first tile as 32-bit words"
    )
    ap.add_argument("file",
        help="Path to raw .apv file (RFC 9924 Appendix A format)")
    ap.add_argument("--tile",  type=int, default=0,
        help="0-based tile index to extract (default: 0)")
    ap.add_argument("--comp",  type=int, default=None,
        help="Extract only this component index (0=Y,1=Cb,2=Cr). "
             "Default: concatenate all components")
    ap.add_argument("--words", type=int, default=None,
        help="Limit output to first N 32-bit words")
    ap.add_argument("--cocotb", action="store_true",
        help="Print a Python list literal for direct import in cocotb")
    ap.add_argument("--no-header-strip", action="store_true",
        help="Include tile_header bytes in the word stream "
             "(default: strip tile_header, emit only entropy-coded data)")
    args = ap.parse_args()

    result = parse_apv_file(args.file, target_tile=args.tile)
    fi = result['frame_info']
    ti = result['tile_info']
    th = result['tile_header']
    off = result['offsets']

    print_frame_info(fi)
    print()
    print_tile_info(ti)
    print()
    print_tile_header(th)

    print(f"\n  pbu_type           : {result['pbu_type']}")
    print(f"  group_id           : {result['group_id']}")
    print(f"  tile_size[{args.tile}]       : {result['tile_size_field']} bytes")
    print(f"  tile @ file offset : 0x{off['tile_size_field']:08X}"
          f"  (tile_data @ 0x{off['tile_data_start']:08X})")

    # Choose bytes to serialise
    if args.no_header_strip:
        # Reconstruct tile payload from tile_header start
        # (tile_header_size tells us how many header bytes there are)
        th_bytes = th['tile_header_size']
        comp_all = b''.join(result['tile_comp_data'])
        raw_bytes = bytes(th_bytes) + comp_all  # header placeholder + data
        label = f"tile {args.tile} full payload (header + all comps)"
    elif args.comp is not None:
        raw_bytes = result['tile_comp_data'][args.comp]
        label = f"tile {args.tile} comp[{args.comp}] entropy data"
    else:
        raw_bytes = b''.join(result['tile_comp_data'])
        label = f"tile {args.tile} entropy data (all comps, header stripped)"

    words = bytes_to_words32(raw_bytes, pad=True)
    if args.words:
        words = words[:args.words]

    if args.cocotb:
        varname = f"TILE{args.tile}_WORDS"
        if args.comp is not None:
            varname = f"TILE{args.tile}_COMP{args.comp}_WORDS"
        print()
        print(cocotb_vector_string(words, label=varname))
    else:
        print_words(words, label=label)

    print(f"\nTotal bytes : {len(raw_bytes)}")
    print(f"Total words : {len(words)} (zero-padded to 32-bit boundary)")


if __name__ == "__main__":
    main()
