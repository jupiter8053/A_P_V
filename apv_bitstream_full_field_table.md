# APV (ISO/IEC 23094-10 / RFC 9924) — Consolidated Bitstream Field Table

One flat table, in bitstream order, covering every syntax element from AU down to
transform-coefficient level. Sizes are exact for fixed fields; variable (`h(v)`) fields
show a bit-range with notes — see the VLC appendix at the end for the exact algorithm.

All multi-bit fields big-endian, MSB first. `u(n)`=unsigned int, `f(n)`=fixed pattern,
`b(8)`=raw byte, `h(v)`=variable-length entropy code.

| # | Container | Field | Type | Size (bits) | Occurs | Notes |
|---|---|---|---|---|---|---|
| 1 | access_unit | `signature` | f(32) | 32 | once | = `'aPv1'` = `0x61507631` |
| 2 | access_unit | `pbu_size` | u(32) | 32 | per PBU | byte size of following `pbu()`; 0 prohibited |
| 3 | pbu_header | `pbu_type` | u(8) | 8 | once | 1/2/25-27=frame, 65=au_info, 66=metadata, 67=filler |
| 4 | pbu_header | `group_id` | u(16) | 16 | once | `0xFFFF` reserved |
| 5 | pbu_header | `reserved_zero_8bits` | u(8) | 8 | once | =0 |
| 6 | frame | `tile_size[i]` | u(32) | 32 | ×NumTiles | byte size of `tile(i)`; 0 reserved |
| 7 | frame_info | `profile_idc` | u(8) | 8 | once | |
| 8 | frame_info | `level_idc` | u(8) | 8 | once | |
| 9 | frame_info | `band_idc` | u(3) | 3 | once | range 0-3 |
| 10 | frame_info | `reserved_zero_5bits` | u(5) | 5 | once | =0 |
| 11 | frame_info | `frame_width` | u(24) | 24 | once | luma samples; 0 reserved |
| 12 | frame_info | `frame_height` | u(24) | 24 | once | luma samples; 0 reserved |
| 13 | frame_info | `chroma_format_idc` | u(4) | 4 | once | 0/2/3/4 valid |
| 14 | frame_info | `bit_depth_minus8` | u(4) | 4 | once | range 2-8 |
| 15 | frame_info | `capture_time_distance` | u(8) | 8 | once | ms |
| 16 | frame_info | `reserved_zero_8bits` | u(8) | 8 | once | =0 |
| 17 | frame_header | `reserved_zero_8bits` | u(8) | 8 | once | =0 |
| 18 | frame_header | `color_description_present_flag` | u(1) | 1 | once | |
| 19 | frame_header | `color_primaries` | u(8) | 8 | if #18==1 | ITU-T H.273; default 2 |
| 20 | frame_header | `transfer_characteristics` | u(8) | 8 | if #18==1 | ITU-T H.273; default 2 |
| 21 | frame_header | `matrix_coefficients` | u(8) | 8 | if #18==1 | ITU-T H.273; default 2 |
| 22 | frame_header | `full_range_flag` | u(1) | 1 | if #18==1 | default 0 |
| 23 | frame_header | `use_q_matrix` | u(1) | 1 | once | |
| 24 | quantization_matrix | `q_matrix[i][x][y]` | u(8) | 8 | ×(NumComps×64) if #23==1 | default 16 if absent; 0 reserved |
| 25 | tile_info | `tile_width_in_mbs` | u(20) | 20 | once | |
| 26 | tile_info | `tile_height_in_mbs` | u(20) | 20 | once | |
| 27 | tile_info | `tile_size_present_in_fh_flag` | u(1) | 1 | once | |
| 28 | tile_info | `tile_size_in_fh[i]` | u(32) | 32 | ×NumTiles if #27==1 | MUST equal `tile_size[i]` (#6) |
| 29 | frame_header | `reserved_zero_8bits` | u(8) | 8 | once | =0 |
| 30 | frame_header | `alignment_bit_equal_to_zero` | f(1) | **0–7** | 0+ | pads to byte boundary; each bit =0 |
| 31 | au_info | `num_frames` | u(16) | 16 | once | (only if pbu_type==65) |
| 32 | au_info | `pbu_type` | u(8) | 8 | ×num_frames | MUST be 1/2/25/26/27 |
| 33 | au_info | `group_id` | u(16) | 16 | ×num_frames | |
| 34 | au_info | `reserved_zero_8bits` | u(8) | 8 | ×num_frames | =0 |
| 35 | au_info | `frame_info()` | — | 96 | ×num_frames | = rows 7–16 |
| 36 | au_info | `reserved_zero_8bits` | u(8) | 8 | once | =0 |
| 37 | metadata | `metadata_size` | u(32) | 32 | once | (only if pbu_type==66) |
| 38 | metadata | `ff_byte` (type ext.) | f(8) | 8 | 0+ while next byte==0xFF | extends payload type |
| 39 | metadata | `metadata_payload_type` | u(8) | 8 | once | last byte of payload type |
| 40 | metadata | `ff_byte` (size ext.) | f(8) | 8 | 0+ while next byte==0xFF | extends payload size |
| 41 | metadata | `metadata_payload_size` | u(8) | 8 | once | last byte of payload size |
| 42 | metadata | `metadata_payload()` | — | **payloadSize × 8** | once | see RFC 9924 §8 |
| 43 | filler | `ff_byte` | f(8) | 8 | 0+ while next byte==0xFF | value always `0xFF` |
| 44 | tile_header | `tile_header_size` | u(16) | 16 | once | bytes |
| 45 | tile_header | `tile_index` | u(16) | 16 | once | MUST == raster-scan tile index |
| 46 | tile_header | `tile_data_size[i]` | u(32) | 32 | ×NumComps | per-component coded data size; 0 reserved |
| 47 | tile_header | `tile_qp[i]` | u(8) | 8 | ×NumComps | `Qp[i]=tile_qp[i]-QpBdOffset`, range -QpBdOffset..51 |
| 48 | tile_header | `reserved_zero_8bits` | u(8) | 8 | once | =0 |
| 49 | tile_header | `alignment_bit_equal_to_zero` | f(1) | **0–7** | 0+ | pads to byte boundary |
| 50 | tile_data | `macroblock_layer()` | — | sum of rows 51–54 | ×numMbsInTile | per component |
| 51 | macroblock_layer | `abs_dc_coeff_diff` | **h(v)** | **1–31** | ×1 per 8×8 block | see VLC appendix; value range 0..32768 |
| 52 | macroblock_layer | `sign_dc_coeff_diff` | u(1) | 1 | if #51 != 0 | 0=+, 1=- |
| 53 | ac_coeff_coding | `coeff_zero_run` | **h(v)** | **1–13** | ×1 per AC iteration | value range 0..62 (block has 64 positions) |
| 54 | ac_coeff_coding | `abs_ac_coeff_minus1` | **h(v)** | **1–31** | if scanPos<blockSize | value range 0..32767 |
| 55 | ac_coeff_coding | `sign_ac_coeff` | u(1) | 1 | if scanPos<blockSize | 0=+, 1=- |
| 56 | tile | `tile_dummy_byte` | b(8) | 8 | 0+ (pad to `tile_size[i]`) | any pattern |
| 57 | (generic) | `alignment_bit_equal_to_zero` | f(1) | **0–7** | 0+ | `byte_alignment()`, used at #30/#49 and after `tile_data`/`au_info` |

---

## Per-MB block counts (for rows 50–55)

8×8 transform blocks (`TrSize=8`) per macroblock, per component:

| Component | chroma_format_idc | Blocks/MB |
|---|---|---|
| Luma (cIdx=0) | any | 4 |
| Chroma (cIdx=1,2), 4:2:2 | 2 | 2 |
| Chroma (cIdx=1,2), 4:4:4 / 4:4:4:4 | 3, 4 | 4 |
| 4th comp (cIdx=3), 4:4:4:4 | 4 | 4 |

Each 8×8 block = rows 51–52 once, then rows 53–55 repeated until 63 AC positions are filled.

---

## VLC appendix — h(v) parsing algorithm (rows 51, 53, 54)

All three `h(v)` elements (`abs_dc_coeff_diff`, `coeff_zero_run`, `abs_ac_coeff_minus1`) use
the **same generic parsing process**, parameterized by a context-derived `kParam`
(an adaptive order, derived from previously decoded values — `PrevDcDiff`, `PrevRun`,
`PrevLevel`/`Prev1stAcLevel` — per RFC 9924 §7.1.1–7.1.3). The generic process:

```
symbolValue = 0
parseExpGolomb = 1
k = kParam
stopLoop = 0

if (read_bits(1) == 1) {
    parseExpGolomb = 0                  // symbolValue stays 0
} else {
    if (read_bits(1) == 0) {
        symbolValue += (1 << k)
        parseExpGolomb = 0
    } else {
        symbolValue += (2 << k)
        parseExpGolomb = 1
    }
}

if (parseExpGolomb) {
    do {
        if (read_bits(1) == 1) {
            stopLoop = 1
        } else {
            symbolValue += (1 << k)
            k++
        }
    } while (!stopLoop)
}

if (k > 0)
    symbolValue += read_bits(k)
```

### Bit-length vs. symbolValue (for a given kParam = k)

| Total bits read | symbolValue range | Prefix pattern |
|---|---|---|
| `1 + k` | `0 .. 2^k - 1` | `1` then k-bit suffix |
| `2 + k` | `2^k .. 2^(k+1) - 1` | `01` + `0` then k-bit suffix |
| `2m + 1 + k` (m≥1) | growing ranges, size `2^(m-1)` each | `01` + `(m-1)` zeros + `1`, then `(k+m-1)`-bit suffix |

### Practical ranges used in the table above

| Field | Value domain | k=0 min bits | k=0 max bits |
|---|---|---|---|
| `abs_dc_coeff_diff` | 0 .. 32768 (`TransCoeff` is 16-bit signed) | 1 | ~31 |
| `abs_ac_coeff_minus1` | 0 .. 32767 | 1 | ~31 |
| `coeff_zero_run` | 0 .. 62 (8×8 block = 64 scan positions, scanPos starts at 1) | 1 | ~13 |

`kParam` adapts upward for larger previous values, which shortens the codeword for
subsequent large values (trading off the `1+k`/`2+k` baseline against fewer loop
iterations). **A bit-accurate parser must implement the state machine above with the
live `kParam` derivation — it cannot use a static lookup table.** The ranges here bound
what your shift-register / barrel-shifter needs to handle per cycle (1 to ~31 bits per
symbol, plus a 1-bit sign for two of the three).
