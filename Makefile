# Makefile for apv_dec_coeff cocotb testbench 

SIM           ?= questa
TOPLEVEL_LANG  = vhdl

VHDL_SOURCES   = $(PWD)/rtl/pkg_apv.vhd          \
                 $(PWD)/rtl/apv_dec_shifter.vhd   \
                 $(PWD)/rtl/apv_dec_coeff.vhd

TOPLEVEL       = apv_dec_coeff
MODULE         = tb.test_apv_dec_coeff

# Pass QP to testbench via environment variable 
ifdef APV_QP
  export APV_QP
endif

ifeq ($(SIM),nvc)
  NVC_ARGS    += --std=93
endif

ifeq ($(SIM),questa)
  VCOM_ARGS += -93
  ifeq ($(WAVES),1)
    SIM_ARGS += -do "log -r /*; run -all; quit"
  endif
endif

include $(shell cocotb-config --makefiles)/Makefile.sim

# Utility targets 
.PHONY: clean_all


clean_all: clean
	rm -f sim.ghw dump.vcd
