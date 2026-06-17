-- =============================================================================
-- apv_fdct1d_core.vhd  —  Shared APV 8-point FDCT butterfly CORE, NO DSP,
--                          NO internal shift/round/clip
-- =============================================================================
-- This is the no-DSP shift-add butterfly (S1 E/O, S2 EE/EO combine, S3A/B/C
-- shift-add multiplies) with the final round+shift+clip stage REMOVED. The
-- core always does the exact same thing every time it runs — no flag, no
-- mode, no enable — which is what makes it safe to share between the row
-- pass and the column pass: it has no notion of "which pass" at all.
--
-- Output is the RAW accumulator for each of the 8 outputs, full width,
-- untouched (no shift, no rounding constant added, no clipping). Width:
--   RAW_W = IN_W + 9   (worst-case verified; IN_W=16 -> RAW_W=25)
--
-- Use apv_round_shift_clip.vhd (separate, tiny module) OUTSIDE this core to
-- finish each pass:
--   Row pass    (after pass 1, before transpose): SHIFT_AMT=>4
--   Column pass (final fdct2d output):            SHIFT_AMT=>9
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity apv_fdct1d_core is
    generic (
        IN_W  : integer := 16    -- fixed/shared input sample width
    );
    port (
        clk        : in  std_logic;
        rst_n      : in  std_logic;
        din_valid  : in  std_logic;
        din        : in  std_logic_vector(IN_W*8-1 downto 0);
        dout_valid : out std_logic;
        -- 8 RAW accumulators, packed, RAW_W bits each, NO shift/round/clip applied
        dout       : out std_logic_vector((IN_W+9)*8-1 downto 0)
    );
end entity apv_fdct1d_core;

architecture rtl of apv_fdct1d_core is

    constant EW    : integer := IN_W + 1;
    constant EEW   : integer := IN_W + 2;
    constant MQW   : integer := EW  + 7;
    constant MEW   : integer := EEW + 7;
    constant RAW_W : integer := IN_W + 9;   -- verified worst-case raw output width

    subtype e_t    is signed(EW-1   downto 0);
    subtype ee_t   is signed(EEW-1  downto 0);
    subtype mulq_t is signed(MQW-1  downto 0);
    subtype mule_t is signed(MEW-1  downto 0);
    subtype raw_t  is signed(RAW_W-1 downto 0);

    type e_arr   is array (0 to 3) of e_t;
    type raw_arr is array (0 to 7) of raw_t;
    type q_arr_t is array (0 to 3) of mulq_t;

    signal s1_valid : std_logic := '0';
    signal Es, Os    : e_arr;

    signal s2_valid  : std_logic := '0';
    signal EE0, EE1, EO0, EO1 : ee_t;
    signal O2 : e_arr;

    signal s3a_valid : std_logic := '0';
    signal eo0_x64, eo0_x16, eo0_x4 : mule_t;
    signal eo0_x32, eo0_x2, eo0_x1  : mule_t;
    signal eo1_x64, eo1_x16, eo1_x4 : mule_t;
    signal eo1_x32, eo1_x2, eo1_x1  : mule_t;
    signal s3a_EE0, s3a_EE1 : ee_t;
    signal o_x64, o_x32 : q_arr_t;
    signal o_x16, o_x8  : q_arr_t;
    signal o_x4,  o_x2  : q_arr_t;
    signal o_x1         : q_arr_t;

    signal s3b_valid : std_logic := '0';
    signal eo0_t84, eo1_t84   : mule_t;
    signal eo0_rem4, eo1_rem4 : mule_t;
    signal eo0_t35, eo1_t35   : mule_t;
    signal eo0_rem1, eo1_rem1 : mule_t;
    signal s3b_EE0, s3b_EE1   : ee_t;
    signal o_t89, o_t75 : q_arr_t;
    signal o_r89, o_r75 : q_arr_t;
    signal o_t50        : q_arr_t;
    signal o_r50        : q_arr_t;
    signal o_mul18      : q_arr_t;

    signal s3c_valid : std_logic := '0';
    signal m84_eo0, m35_eo1 : mule_t;
    signal m35_eo0, m84_eo1 : mule_t;
    signal s3c_EE0, s3c_EE1 : ee_t;
    signal m89, m75, m50, m18 : q_arr_t;

    -- ═══════════════════════════════════════════════════════════════════════
    -- S4 — accumulate ONLY (no round, no shift, no clip) — raw output
    -- ═══════════════════════════════════════════════════════════════════════
    signal s4_valid : std_logic := '0';
    signal raw       : raw_arr;

begin

    process(clk)
        type in8_t is array (0 to 7) of signed(IN_W-1 downto 0);
        variable src : in8_t;
    begin
        if rising_edge(clk) then
            s1_valid <= din_valid;
            if din_valid = '1' then
                for k in 0 to 7 loop
                    src(k) := signed(din(IN_W*(k+1)-1 downto IN_W*k));
                end loop;
                for k in 0 to 3 loop
                    Es(k) <= resize(src(k), EW) + resize(src(7-k), EW);
                    Os(k) <= resize(src(k), EW) - resize(src(7-k), EW);
                end loop;
            end if;
        end if;
    end process;

    process(clk)
    begin
        if rising_edge(clk) then
            s2_valid <= s1_valid;
            if s1_valid = '1' then
                EE0 <= resize(Es(0),EEW) + resize(Es(3),EEW);
                EO0 <= resize(Es(0),EEW) - resize(Es(3),EEW);
                EE1 <= resize(Es(1),EEW) + resize(Es(2),EEW);
                EO1 <= resize(Es(1),EEW) - resize(Es(2),EEW);
                O2  <= Os;
            end if;
        end if;
    end process;

    process(clk)
    begin
        if rising_edge(clk) then
            s3a_valid <= s2_valid;
            if s2_valid = '1' then
                eo0_x64 <= shift_left(resize(EO0, MEW), 6);
                eo0_x16 <= shift_left(resize(EO0, MEW), 4);
                eo0_x4  <= shift_left(resize(EO0, MEW), 2);
                eo0_x32 <= shift_left(resize(EO0, MEW), 5);
                eo0_x2  <= shift_left(resize(EO0, MEW), 1);
                eo0_x1  <= resize(EO0, MEW);
                eo1_x64 <= shift_left(resize(EO1, MEW), 6);
                eo1_x16 <= shift_left(resize(EO1, MEW), 4);
                eo1_x4  <= shift_left(resize(EO1, MEW), 2);
                eo1_x32 <= shift_left(resize(EO1, MEW), 5);
                eo1_x2  <= shift_left(resize(EO1, MEW), 1);
                eo1_x1  <= resize(EO1, MEW);
                s3a_EE0 <= EE0;
                s3a_EE1 <= EE1;
                for k in 0 to 3 loop
                    o_x64(k) <= shift_left(resize(O2(k), MQW), 6);
                    o_x32(k) <= shift_left(resize(O2(k), MQW), 5);
                    o_x16(k) <= shift_left(resize(O2(k), MQW), 4);
                    o_x8(k)  <= shift_left(resize(O2(k), MQW), 3);
                    o_x4(k)  <= shift_left(resize(O2(k), MQW), 2);
                    o_x2(k)  <= shift_left(resize(O2(k), MQW), 1);
                    o_x1(k)  <= resize(O2(k), MQW);
                end loop;
            end if;
        end if;
    end process;

    process(clk)
    begin
        if rising_edge(clk) then
            s3b_valid <= s3a_valid;
            if s3a_valid = '1' then
                eo0_t84  <= eo0_x64 + eo0_x16;
                eo0_rem4 <= eo0_x4;
                eo1_t84  <= eo1_x64 + eo1_x16;
                eo1_rem4 <= eo1_x4;
                eo0_t35  <= eo0_x32 + eo0_x2;
                eo0_rem1 <= eo0_x1;
                eo1_t35  <= eo1_x32 + eo1_x2;
                eo1_rem1 <= eo1_x1;
                s3b_EE0 <= s3a_EE0;
                s3b_EE1 <= s3a_EE1;
                for k in 0 to 3 loop
                    o_t89(k) <= o_x64(k) + o_x16(k);
                    o_r89(k) <= o_x8(k)  + o_x1(k);
                end loop;
                for k in 0 to 3 loop
                    o_t75(k) <= o_x64(k) + o_x8(k);
                    o_r75(k) <= o_x2(k)  + o_x1(k);
                end loop;
                for k in 0 to 3 loop
                    o_t50(k) <= o_x32(k) + o_x16(k);
                    o_r50(k) <= o_x2(k);
                end loop;
                for k in 0 to 3 loop
                    o_mul18(k) <= o_x16(k) + o_x2(k);
                end loop;
            end if;
        end if;
    end process;

    process(clk)
    begin
        if rising_edge(clk) then
            s3c_valid <= s3b_valid;
            if s3b_valid = '1' then
                m84_eo0 <= eo0_t84 + eo0_rem4;
                m84_eo1 <= eo1_t84 + eo1_rem4;
                m35_eo0 <= eo0_t35 + eo0_rem1;
                m35_eo1 <= eo1_t35 + eo1_rem1;
                s3c_EE0 <= s3b_EE0;
                s3c_EE1 <= s3b_EE1;
                for k in 0 to 3 loop
                    m89(k) <= o_t89(k) + o_r89(k);
                end loop;
                for k in 0 to 3 loop
                    m75(k) <= o_t75(k) + o_r75(k);
                end loop;
                for k in 0 to 3 loop
                    m50(k) <= o_t50(k) + o_r50(k);
                end loop;
                for k in 0 to 3 loop
                    m18(k) <= o_mul18(k);
                end loop;
            end if;
        end if;
    end process;

    -- ════════════════════════════════════════════════════════════════════════
    -- S4 — accumulate ONLY. No (+ADD), no (>>SHIFT), no clip. Raw output.
    -- ════════════════════════════════════════════════════════════════════════
    process(clk)
        variable sum_ee : signed(EEW downto 0);
    begin
        if rising_edge(clk) then
            s4_valid <= s3c_valid;
            if s3c_valid = '1' then

                sum_ee := resize(s3c_EE0, EEW+1) + resize(s3c_EE1, EEW+1);
                raw(0) <= resize(shift_left(resize(sum_ee, RAW_W), 6), RAW_W);

                sum_ee := resize(s3c_EE0, EEW+1) - resize(s3c_EE1, EEW+1);
                raw(4) <= resize(shift_left(resize(sum_ee, RAW_W), 6), RAW_W);

                raw(2) <= resize(m84_eo0, RAW_W) + resize(m35_eo1, RAW_W);
                raw(6) <= resize(m35_eo0, RAW_W) - resize(m84_eo1, RAW_W);

                raw(1) <= resize(m89(0),RAW_W) + resize(m75(1),RAW_W)
                        + resize(m50(2),RAW_W) + resize(m18(3),RAW_W);

                raw(3) <= resize(m75(0),RAW_W) - resize(m18(1),RAW_W)
                        - resize(m89(2),RAW_W) - resize(m50(3),RAW_W);

                raw(5) <= resize(m50(0),RAW_W) - resize(m89(1),RAW_W)
                        + resize(m18(2),RAW_W) + resize(m75(3),RAW_W);

                raw(7) <= resize(m18(0),RAW_W) - resize(m50(1),RAW_W)
                        + resize(m75(2),RAW_W) - resize(m89(3),RAW_W);

            end if;
        end if;
    end process;

    GEN_PACK: for k in 0 to 7 generate
        dout(RAW_W*(k+1)-1 downto RAW_W*k) <= std_logic_vector(raw(k));
    end generate GEN_PACK;
    dout_valid <= s4_valid;

end architecture rtl;
