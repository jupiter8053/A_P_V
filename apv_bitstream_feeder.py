#!/usr/bin/env python3
"""
apv_bitstream_feeder.py
-----------------------
Parse an APV (.apv) file, extract the first tile's raw bitstream,
and yield it as 32-bit big-endian words for cocotb simulation.

APV bitstream structure (RFC 9924 / ISO 23094-10):
  [frame_info_size : 24] [frame_info : frame_info_size bytes]
    frame_info contains:
      [pbu_type : 8]  [reserved : 8]  [frame_size : 32]
      [profile_idc : 8]  [level_idc : 8]  [band_idc : 4]
      [reserved : 6]  [color_description_present_flag : 1]
      [capture_time_distance_ignored : 1]
      ... chroma/bit-depth fields ...
      [num_comp : 4]  [comp_info[i].width_in_mbs : 16] ...
      [tile_width_in_mbs : 16]  [tile_height_in_mbs : 16]
    => tile grid = ceil(frame_width/mb) x ceil(frame_height/mb)

  After frame_info block: tile data
    For each tile (row-major):
      [tile_size : 32]  (bytes that follow, NOT including this 4-byte field)
      [tile_header : variable]
        [tile_index : 16]
        [tile_data_size[comp] : 32] x num_comp
      [comp_data[comp] : tile_data_size[comp] bytes] x num_comp

Usage:
  python apv_bitstream_feeder.py <file.apv> [--tile 0] [--hex] [--words N]
  python apv_bitstream_feeder.py <file.apv> --cocotb  (prints loadable vector)
"""

import struct
import sys
import argparse
import math


# ---------------------------------------------------------------------------
# Low-level bit reader
# ---------------------------------------------------------------------------

class BitReader:
    """Read individual bits, bytes, and multi-bit fields from a bytes object."""

    def __init__(self, data: bytes):
        self._data = data
        self._byte_pos = 0
        self._bit_pos = 0          # 0 = MSB of current byte

    @property
    def byte_pos(self) -> int:
        return self._byte_pos

    @property
    def bits_consumed(self) -> int:
        return self._byte_pos * 8 + self._bit_pos

    def bytes_consumed(self) -> int:
        return self._byte_pos + (1 if self._bit_pos > 0 else 0)

    def _available_bits(self) -> int:
        return (len(self._data) - self._byte_pos) * 8 - self._bit_pos

    def read_bits(self, n: int) -> int:
        """Read n bits, MSB first. Returns unsigned integer."""
        if n == 0:
            return 0
        if self._available_bits() < n:
            raise EOFError(
                f"Need {n} bits but only {self._available_bits()} remain "
                f"(byte_pos={self._byte_pos}, bit_pos={self._bit_pos})"
            )
        result = 0
        remaining = n
        while remaining > 0:
            bits_in_current_byte = 8 - self._bit_pos
            take = min(bits_in_current_byte, remaining)
            shift = bits_in_current_byte - take
            mask = ((1 << take) - 1)
            bits = (self._data[self._byte_pos] >> shift) & mask
            result = (result << take) | bits
            self._bit_pos += take
            remaining -= take
            if self._bit_pos == 8:
                self._byte_pos += 1
                self._bit_pos = 0
        return result

    def read_u8(self) -> int:
        return self.read_bits(8)

    def read_u16(self) -> int:
        return self.read_bits(16)

    def read_u32(self) -> int:
        return self.read_bits(32)

    def read_u24(self) -> int:
        return self.read_bits(24)

    def align_to_byte(self):
        """Skip remaining bits in the current byte."""
        if self._bit_pos != 0:
            self._byte_pos += 1
            self._bit_pos = 0

    def read_bytes(self, n: int) -> bytes:
        """Read n whole bytes (must be byte-aligned)."""
        self.align_to_byte()
        start = self._byte_pos
        self._byte_pos += n
        return self._data[start:self._byte_pos]

    def skip_bytes(self, n: int):
        self.align_to_byte()
        self._byte_pos += n


# ---------------------------------------------------------------------------
# APV frame_info parser  (Section 4.4 of RFC 9924)
# ---------------------------------------------------------------------------

def parse_frame_info(br: BitReader) -> dict:
    """
    Parse the frame_info() syntax element.
    Returns a dict with all scalar fields plus derived tile grid dimensions.
    """
    fi = {}

    fi['pbu_type']          = br.read_u8()
    fi['reserved0']         = br.read_u8()   # shall be 0
    fi['frame_size']        = br.read_u32()  # bytes in this PBU

    fi['profile_idc']       = br.read_u8()
    fi['level_idc']         = br.read_u8()
    fi['band_idc']          = br.read_bits(4)
    fi['reserved1']         = br.read_bits(6)
    fi['color_description_present_flag'] = br.read_bits(1)
    fi['capture_time_distance_ignored']  = br.read_bits(1)

    fi['color_primaries']         = 0
    fi['transfer_characteristics'] = 0
    fi['matrix_coefficients']      = 0
    fi['full_range_flag']          = 0
    if fi['color_description_present_flag']:
        fi['color_primaries']          = br.read_u8()
        fi['transfer_characteristics'] = br.read_u8()
        fi['matrix_coefficients']      = br.read_u8()
        fi['full_range_flag']          = br.read_bits(1)
        br.read_bits(7)                # reserved

    fi['num_comp']         = br.read_bits(4)
    fi['reserved2']        = br.read_bits(4)
    fi['bit_depth']        = br.read_u8() + 1   # stored as (bit_depth - 1)

    num_comp = fi['num_comp']
    fi['comp_info'] = []
    for _ in range(num_comp):
        comp = {}
        comp['width']       = br.read_u16() + 1  # stored as (w - 1)
        comp['height']      = br.read_u16() + 1  # stored as (h - 1)
        comp['h_samp']      = br.read_u8()        # 0=444,1=422,2=420
        comp['v_samp']      = br.read_u8()
        fi['comp_info'].append(comp)

    fi['tile_width_in_mbs']  = br.read_u16() + 1
    fi['tile_height_in_mbs'] = br.read_u16() + 1
    fi['reserved3']          = br.read_u16()      # shall be 0

    # Derive tile grid from luma component (comp 0) dimensions
    # Each MB (macroblock) is 16x16 luma pixels
    MB = 16
    luma_w = fi['comp_info'][0]['width']
    luma_h = fi['comp_info'][0]['height']
    tw_px  = fi['tile_width_in_mbs']  * MB
    th_px  = fi['tile_height_in_mbs'] * MB
    fi['num_tiles_cols'] = math.ceil(luma_w / tw_px)
    fi['num_tiles_rows'] = math.ceil(luma_h / th_px)
    fi['num_tiles']      = fi['num_tiles_cols'] * fi['num_tiles_rows']

    return fi


# ---------------------------------------------------------------------------
# APV tile header parser  (Section 4.6 of RFC 9924)
# ---------------------------------------------------------------------------

def parse_tile_header(br: BitReader, num_comp: int) -> dict:
    """
    Parse the tile_header() syntax element (inside the tile payload).
    Returns dict with tile_index and per-component data sizes.
    """
    th = {}
    th['tile_index']      = br.read_u16()
    th['reserved']        = br.read_u16()
    th['comp_data_size']  = []
    for _ in range(num_comp):
        th['comp_data_size'].append(br.read_u32())
    return th


# ---------------------------------------------------------------------------
# Main APV file parser
# ---------------------------------------------------------------------------

def parse_apv_file(path: str, target_tile: int = 0) -> dict:
    """
    Open an APV file, locate the target tile (0-based), and return:
      {
        'frame_info': dict,
        'tile_header': dict,
        'tile_payload_bytes': bytes,   # full tile payload (header + comp data)
        'tile_comp_data': list[bytes], # per-component entropy-coded data
        'tile_offset': int,            # byte offset in file where tile_size field begins
        'tile_size': int,              # value of the 4-byte tile_size field
      }
    """
    with open(path, 'rb') as f:
        raw = f.read()

    # -----------------------------------------------------------------------
    # Step 1: parse frame_info_size + frame_info
    # -----------------------------------------------------------------------
    # The file begins with: [frame_info_size : 3 bytes BE] [frame_info : N bytes]
    if len(raw) < 3:
        raise ValueError("File too short to be a valid APV bitstream")

    fi_size = struct.unpack_from('>I', b'\x00' + raw[0:3])[0]  # 24-bit BE
    fi_data = raw[3 : 3 + fi_size]
    if len(fi_data) < fi_size:
        raise ValueError(f"frame_info truncated: expected {fi_size} bytes")

    br_fi = BitReader(fi_data)
    frame_info = parse_frame_info(br_fi)

    # -----------------------------------------------------------------------
    # Step 2: walk tile list to find target_tile
    # -----------------------------------------------------------------------
    tile_start = 3 + fi_size          # first tile_size field byte offset in file
    num_tiles  = frame_info['num_tiles']
    num_comp   = frame_info['num_comp']

    if target_tile >= num_tiles:
        raise ValueError(
            f"Requested tile {target_tile} but frame only has {num_tiles} tiles"
        )

    pos = tile_start
    for tile_idx in range(num_tiles):
        if pos + 4 > len(raw):
            raise ValueError(f"Truncated APV file at tile {tile_idx}")

        tile_size_field = struct.unpack_from('>I', raw, pos)[0]
        tile_data_start = pos + 4
        tile_data_end   = tile_data_start + tile_size_field

        if tile_data_end > len(raw):
            raise ValueError(
                f"Tile {tile_idx}: tile_size={tile_size_field} extends past EOF"
            )

        if tile_idx == target_tile:
            tile_payload = raw[tile_data_start : tile_data_end]

            # Parse tile header from payload
            br_th = BitReader(tile_payload)
            tile_header = parse_tile_header(br_th, num_comp)
            br_th.align_to_byte()
            header_bytes = br_th.byte_pos

            # Extract per-component entropy-coded data
            comp_data = []
            comp_pos = header_bytes
            for c in range(num_comp):
                sz = tile_header['comp_data_size'][c]
                comp_data.append(tile_payload[comp_pos : comp_pos + sz])
                comp_pos += sz

            return {
                'frame_info':        frame_info,
                'tile_header':       tile_header,
                'tile_payload_bytes': tile_payload,
                'tile_comp_data':    comp_data,
                'tile_offset':       pos,
                'tile_size':         tile_size_field,
                'frame_info_size':   fi_size,
                'tile_data_start':   tile_data_start,
            }

        # Advance to next tile
        pos = tile_data_end

    raise RuntimeError(f"Tile {target_tile} not found (walked {num_tiles} tiles)")


# ---------------------------------------------------------------------------
# 32-bit word serialiser
# ---------------------------------------------------------------------------

def bytes_to_words32(data: bytes, pad: bool = True) -> list[int]:
    """
    Pack bytes into 32-bit big-endian words.
    If pad=True, zero-pad the last word to a full 4 bytes.
    """
    if pad:
        rem = len(data) % 4
        if rem:
            data = data + b'\x00' * (4 - rem)
    words = []
    for i in range(0, len(data), 4):
        w = struct.unpack_from('>I', data, i)[0]
        words.append(w)
    return words


# ---------------------------------------------------------------------------
# Pretty print helpers
# ---------------------------------------------------------------------------

def print_frame_info(fi: dict):
    print("=== frame_info ===")
    print(f"  pbu_type       : {fi['pbu_type']:#04x}")
    print(f"  frame_size     : {fi['frame_size']} bytes")
    print(f"  profile_idc    : {fi['profile_idc']}")
    print(f"  level_idc      : {fi['level_idc']}")
    print(f"  band_idc       : {fi['band_idc']}")
    print(f"  bit_depth      : {fi['bit_depth']}")
    print(f"  num_comp       : {fi['num_comp']}")
    for i, c in enumerate(fi['comp_info']):
        print(f"    comp[{i}]  w={c['width']} h={c['height']} "
              f"h_samp={c['h_samp']} v_samp={c['v_samp']}")
    print(f"  tile_width_in_mbs  : {fi['tile_width_in_mbs']}")
    print(f"  tile_height_in_mbs : {fi['tile_height_in_mbs']}")
    print(f"  tile grid          : {fi['num_tiles_cols']} x "
          f"{fi['num_tiles_rows']} = {fi['num_tiles']} tiles")


def print_tile_header(th: dict):
    print("=== tile_header ===")
    print(f"  tile_index : {th['tile_index']}")
    for i, sz in enumerate(th['comp_data_size']):
        print(f"  comp[{i}] data_size : {sz} bytes")


def print_words(words: list[int], label: str = "words"):
    print(f"\n=== {label} ({len(words)} x 32-bit words) ===")
    for i, w in enumerate(words):
        marker = "<-- first" if i == 0 else ""
        print(f"  [{i:4d}] 0x{w:08X}  {w:032b}  {marker}")


# ---------------------------------------------------------------------------
# cocotb helper: generate a Python list literal for direct import
# ---------------------------------------------------------------------------

def cocotb_vector_string(words: list[int]) -> str:
    lines = ["TILE0_WORDS = ["]
    for i, w in enumerate(words):
        sep = "," if i < len(words) - 1 else ""
        lines.append(f"    0x{w:08X}{sep}  # [{i}]")
    lines.append("]")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Extract first APV tile and emit 32-bit words for simulation"
    )
    parser.add_argument("file", help="Path to .apv file")
    parser.add_argument(
        "--tile", type=int, default=0,
        help="0-based tile index to extract (default: 0)"
    )
    parser.add_argument(
        "--comp", type=int, default=None,
        help="Extract only this component's data (0=Y,1=Cb,2=Cr). "
             "Default: full tile payload"
    )
    parser.add_argument(
        "--hex", action="store_true",
        help="Print 32-bit words in hex (default: also binary)"
    )
    parser.add_argument(
        "--words", type=int, default=None,
        help="Limit output to first N words"
    )
    parser.add_argument(
        "--cocotb", action="store_true",
        help="Print a Python list literal suitable for direct import in cocotb"
    )
    parser.add_argument(
        "--no-header-strip", action="store_true",
        help="Include tile_header bytes in the output word stream "
             "(default: strip tile_header, emit only entropy-coded data)"
    )
    args = parser.parse_args()

    result = parse_apv_file(args.file, target_tile=args.tile)
    fi = result['frame_info']
    th = result['tile_header']

    print_frame_info(fi)
    print()
    print_tile_header(th)

    print(f"\n  tile @ file offset 0x{result['tile_offset']:08X} "
          f"(tile_size field = {result['tile_size']} bytes)")
    print(f"  tile_data_start  = 0x{result['tile_data_start']:08X}")

    # Choose data to serialise
    if args.comp is not None:
        raw_bytes = result['tile_comp_data'][args.comp]
        label = f"tile {args.tile} comp[{args.comp}] entropy data"
    elif args.no_header_strip:
        raw_bytes = result['tile_payload_bytes']
        label = f"tile {args.tile} full payload (header + data)"
    else:
        # Default: concatenated entropy data for all components, no tile_header
        raw_bytes = b''.join(result['tile_comp_data'])
        label = f"tile {args.tile} entropy data (all comps, header stripped)"

    words = bytes_to_words32(raw_bytes, pad=True)
    if args.words:
        words = words[:args.words]

    if args.cocotb:
        print()
        print(cocotb_vector_string(words))
    else:
        print_words(words, label=label)

    # Summary stats
    print(f"\nTotal bytes : {len(raw_bytes)}")
    print(f"Total words : {len(words)} (zero-padded to 32-bit boundary)")


if __name__ == "__main__":
    main()
