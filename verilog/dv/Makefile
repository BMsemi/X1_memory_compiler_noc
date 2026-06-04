# SPDX-License-Identifier: Apache-2.0

.PHONY: cocotb clean

cocotb:
	cd cocotb && caravel_cocotb -t x1_memory_compiler_noc -sim RTL -design_info design_info.yaml -no_wave

clean:
	rm -rf cocotb/sim
