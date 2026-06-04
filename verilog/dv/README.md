# Design Verification

This repository keeps the Caravel DV directory layout but only ships the X1 memory compiler cocotb smoke test.

Run the RTL smoke from erilog/dv/cocotb after creating a machine-local design_info.yaml from design_info.example.yaml:

`ash
caravel_cocotb -t x1_memory_compiler_noc -sim RTL -design_info design_info.yaml -no_wave
`
