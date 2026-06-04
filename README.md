# X1 Memory Compiler NoC Submit

Caravel-format behavioral RTL repository for an X1 ReRAM memory compiler with a small NoC endpoint.

This repo is derived from BMsemi/IMPACT_SNN_RERAM_submit layout conventions, but the design payload is intentionally X1-only:

- uses the latest Neuromorphic_X1_beh behavioral model from BMsemi/IMPACT_SNN_RERAM_submit
- instantiates multiple X1 behavioral macros through x1_memory_compiler_noc
- removes the SNN/adaptive-fabric datapath from the top-level wrapper and RTL file list
- exposes program, reset, read, and compute commands through Wishbone CSRs
- emits FP4LLM-style SAR/TDC packed NoC flits for multi-macro compute results
- includes a Caravel cocotb RTL smoke test for CSR access, X1 readback, and NoC packing

## Repository Layout

| Path | Purpose |
|---|---|
| verilog/rtl/Neuromorphic_X1_Beh.v | X1 behavioral model copied from the IMPACT submit repo |
| verilog/rtl/x1_memory_compiler_noc.v | X1 macro bank, CSR controller, and NoC flit packer |
| verilog/rtl/user_project_wrapper.v | Caravel wrapper integration for the compiler |
| verilog/includes/includes.rtl.caravel_user_project | RTL simulation file list |
| verilog/dv/cocotb/user_proj_tests/x1_memory_compiler_noc/ | Caravel cocotb smoke test |
| docs/x1_memory_compiler_noc.md | CSR map and NoC packet details |

Generated physical views are not included in this behavioral package. The gds/, lef/, lib/, mag/, sdc/, spef/, and verilog/gl/ directories are placeholders until the X1 compiler wrapper is hardened for a physical release.

## RTL Simulation

From verilog/dv/cocotb, copy design_info.example.yaml to design_info.yaml and edit the Caravel, MCW, PDK, and project paths for your machine.

Example command:

~~~bash
caravel_cocotb -t x1_memory_compiler_noc -sim RTL -design_info design_info.yaml -no_wave
~~~

Expected smoke result:

- direct X1 program/readback returns 1
- NoC flit valid/last assert
- role 2 SAR/TDC payload values are [0, 0, 0, 0] for the four default X1 macros under FP4LLM SAR-FP packing

## Scope Notes

This is a behavioral simulation repository. It does not include the original SNN RTL modules or stale wrapper layout generated for the prior design. Before tapeout or GL simulation, regenerate hardening outputs for this wrapper and update the GL include lists accordingly.
