# Nyan Keys - iCE40HX4K build + simulation
#
#   make test      run all self-checking testbenches
#   make sim-<tb>  run a single testbench, e.g. make sim-tb_keys (VCD in build/)
#   make           synthesize, place & route and pack the bitstream
#   make prog      program the FPGA with iceprog
#   make crosscheck FW=../nyan-keys-stm32-firmware
#                  check FPGA frames against the firmware's frame parser

TOP     := spi_keys
DEVICE  := hx4k
PACKAGE := tq144
PCF     := constraints/nyan4k_keys.pcf
CLOCKS  := constraints/clocks.py
SEED    ?= 1

RTL     := rtl/keys.v rtl/spi_frame_tx.v rtl/spi_keys.v
TBS     := tb_keys tb_spi_frame_tx tb_spi_keys
BUILD   := build

IVERILOG := iverilog -g2012 -Wall -Wno-timescale
VVP      := vvp -n

.PHONY: all test prog clean crosscheck $(addprefix sim-,$(TBS))

all: $(BUILD)/$(TOP).bin

$(BUILD):
	mkdir -p $@

# ---------------------------------------------------------------- simulation
$(BUILD)/%.vvp: sim/%.v $(RTL) | $(BUILD)
	$(IVERILOG) -o $@ $< $(RTL)

sim-%: $(BUILD)/%.vvp
	cd $(BUILD) && $(VVP) $*.vvp +vcd | tee $*.log
	@grep -q "^PASS" $(BUILD)/$*.log

test: $(addprefix $(BUILD)/,$(addsuffix .vvp,$(TBS)))
	@fail=0; for tb in $(TBS); do \
		(cd $(BUILD) && $(VVP) $$tb.vvp > $$tb.log 2>&1); \
		if grep -q "^PASS" $(BUILD)/$$tb.log; then echo "PASS $$tb"; \
		else echo "FAIL $$tb (see $(BUILD)/$$tb.log)"; grep ERROR $(BUILD)/$$tb.log | head; fail=1; fi; \
	done; exit $$fail

FW ?= ../nyan-keys-stm32-firmware

crosscheck: $(BUILD)/tb_spi_frame_tx.vvp
	cd $(BUILD) && $(VVP) tb_spi_frame_tx.vvp +dump > /dev/null
	$(CC) -std=c11 -Wall -Wextra -Werror -I$(FW)/Core/Inc -o $(BUILD)/frame_crosscheck sim/frame_crosscheck.c
	$(BUILD)/frame_crosscheck $(BUILD)/frames.hex

# ---------------------------------------------------------------- bitstream
$(BUILD)/$(TOP).json: $(RTL) | $(BUILD)
	yosys -q -l $(BUILD)/yosys.log -p "read_verilog $(RTL); synth_ice40 -top $(TOP) -json $@"

$(BUILD)/$(TOP).asc: $(BUILD)/$(TOP).json $(PCF) $(CLOCKS)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --json $< --pcf $(PCF) \
		--quiet --pre-pack $(CLOCKS) --seed $(SEED) --asc $@ -l $(BUILD)/nextpnr.log

$(BUILD)/$(TOP).bin: $(BUILD)/$(TOP).asc
	icepack $< $@

prog: $(BUILD)/$(TOP).bin
	iceprog $<

clean:
	rm -rf $(BUILD)
