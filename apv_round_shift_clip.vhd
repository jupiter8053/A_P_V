-- =============================================================================
-- apv_round_shift_clip.vhd  —  Finishing stage used OUTSIDE apv_fdct1d_core
-- =============================================================================
-- Takes one raw accumulator value from apv_fdct1d_core and applies the full,
-- complete round+shift+clip for whichever pass is finishing right now.
-- Instantiate this TWICE in your datapath (or reuse via mux — your choice):
--
--   Row pass    (after pass 1, before transpose): SHIFT_AMT => 4
--   Column pass (final fdct2d output):             SHIFT_AMT => 9
--
-- out = clip16( (raw + 2^(SHIFT_AMT-1)) >> SHIFT_AMT )
--
-- No variables inside the process — the round/shift/clip math is done with
-- concurrent signal assignments; the process only registers the result.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity apv_round_shift_clip is
    generic (
        RAW_W     : integer := 25;   -- must match apv_fdct1d_core's RAW_W (IN_W+9)
        OUT_W     : integer := 16;
        SHIFT_AMT : integer := 4     -- 4 for row pass, 9 for column pass
    );
    port (
        clk        : in  std_logic;
        rst_n      : in  std_logic;
        din_valid  : in  std_logic;
        din_raw    : in  std_logic_vector(RAW_W-1 downto 0);
        dout_valid : out std_logic;
        dout       : out std_logic_vector(OUT_W-1 downto 0)
    );
end entity apv_round_shift_clip;

architecture rtl of apv_round_shift_clip is

    -- one extra guard bit for the addition, so raw+ADD can never wrap
    -- regardless of how close raw sits to the RAW_W-bit boundary
    constant SUM_W : integer := RAW_W + 1;
    constant ADD   : signed(SUM_W-1 downto 0) := to_signed(2**(SHIFT_AMT-1), SUM_W);
    constant HI    : signed(SUM_W-1 downto 0) := to_signed(2**(OUT_W-1) - 1, SUM_W);
    constant LO    : signed(SUM_W-1 downto 0) := to_signed(-2**(OUT_W-1),   SUM_W);

    -- combinational: raw -> rounded+shifted -> clipped, all as plain signals
    signal raw      : signed(RAW_W-1 downto 0);
    signal summed   : signed(SUM_W-1 downto 0);   -- raw + ADD, guard-bit wide
    signal shifted  : signed(SUM_W-1 downto 0);
    signal clipped  : signed(OUT_W-1 downto 0);

begin

    raw     <= signed(din_raw);
    summed  <= resize(raw, SUM_W) + ADD;
    shifted <= shift_right(summed, SHIFT_AMT);

    clipped <= to_signed(2**(OUT_W-1) - 1, OUT_W) when shifted > HI else
               to_signed(-2**(OUT_W-1),    OUT_W) when shifted < LO else
               resize(shifted, OUT_W);

    -- process only registers — no variables, just signal <= signal
    process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                dout_valid <= '0';
            else
                dout_valid <= din_valid;
                if din_valid = '1' then
                    dout <= std_logic_vector(clipped);
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
