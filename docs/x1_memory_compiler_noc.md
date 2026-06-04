# X1 Memory Compiler NoC Smoke Model

This repository packages an X1-only behavioral memory compiler around the latest `Neuromorphic_X1_beh` model from `BMsemi/IMPACT_SNN_RERAM_submit`.

## Scope

- Uses only `Neuromorphic_X1_beh` instances from the submit behavioral model.
- Removes the SNN wrapper path from the RTL include list and top-level wrapper.
- Instantiates four X1 behavioral macros by default through `NUM_MACROS`.
- Auto-sends the three submit-model configuration packets after reset before accepting user commands.
- Exposes program, reset, read, and multi-macro compute through a small Wishbone CSR plane.
- Emits one SAR-TDC NoC flit whose 64-bit header follows `Benchmarking/FP4LLM/docs/sar_tdc_noc_network_design.md`.

## CSR Map

| Word Offset | Name | Direction | Description |
|---:|---|---|---|
| 0 | VERSION | R | `0x58314e43` (`X1NC`) |
| 1 | CONFIG | R | macro count and 32x32 X1 tile geometry |
| 2 | STATUS | R | bit0 busy, bit1 init done, bit2 result valid, bit3 NoC valid, bit4 error |
| 3 | COMMAND | W | bit31 start, bits3:0 op, bits7:4 macro id, bit9 full-row compute |
| 4 | ADDR | R/W | bits4:0 row, bits12:8 column |
| 5 | DATA | R/W | bits7:0 program threshold/PWM data |
| 6 | ROLE | R/W | role id, gain shift, VC, destination, tile group |
| 7 | RESULT | R | last raw X1 read/compute word |
| 8-11 | FLIT0..3 | R | NoC flit words, little-endian 32-bit chunks |
| 12 | CLEAR | W | clear result, NoC valid, and/or error sticky bits |

## NoC Format

The generated flit stores `header[63:0]` in `noc_flit_data[63:0]` and the bit-packed SAR-TDC payload in `noc_flit_data[127:64]`. Role policy matches the FP4LLM simulator reference:

| Role ID | Role | Format | Bits/value |
|---:|---|---|---:|
| 1 | gate/up | c4/f3 | 8 |
| 2-7 | down/q/k/v/out/qkt | c5/f3 | 9 |
| 8 | AV | c5/f5 | 11 |

The endpoint packs local X1 macro compute results LSB-first and keeps routing numerical-format agnostic, matching the FP4LLM SAR-TDC dynamic-packing model.
