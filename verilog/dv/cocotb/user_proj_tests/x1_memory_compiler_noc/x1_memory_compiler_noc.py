from caravel_cocotb.caravel_interfaces import test_configure
from caravel_cocotb.caravel_interfaces import report_test
import cocotb
from cocotb.triggers import RisingEdge, ClockCycles, Timer
from cocotb.handle import Force, Release
import json
import math
from pathlib import Path

CSR_STATUS = 2
CSR_COMMAND = 3
CSR_ADDR = 4
CSR_DATA = 5
CSR_ROLE = 6
CSR_RESULT = 7
CSR_FLIT0 = 8
CSR_FLIT1 = 9
CSR_FLIT2 = 10
CSR_FLIT3 = 11

STATUS_BUSY = 0x01
STATUS_INIT = 0x02
STATUS_RESULT = 0x04
STATUS_NOC = 0x08
STATUS_ERROR = 0x10

CMD_START = 0x80000000
CMD_PROGRAM = 0x1
CMD_READ = 0x3
CMD_COMPUTE = 0x4
CMD_FULL_ROW = 1 << 9

FP4LLM_VALUE_COUNT = 32
FP4LLM_AGGREGATE_TILES = 4

def cmd_macro(idx):
    return (idx & 0xF) << 4

def addr_value(row, col):
    return ((col & 0x1F) << 8) | (row & 0x1F)

def role_value(role, gain, vc, dst, tile):
    return ((tile & 0xFFF) << 16) | ((dst & 0xF) << 12) | ((vc & 0x7) << 9) | ((gain & 0x1F) << 4) | (role & 0xF)

def unpack_lsb(payload, count, bits):
    mask = (1 << bits) - 1
    return [(payload >> (idx * bits)) & mask for idx in range(count)]

def role_policy(role_id):
    if role_id == 1:
        return 8, 4, 3
    if role_id == 8:
        return 11, 5, 5
    return 9, 5, 3

def tail_valid_bits(total_bits):
    rem = total_bits & 0x7F
    return 0 if rem == 0 else rem

def decode_header(header):
    return {
        "fmt_id": (header >> 60) & 0xF,
        "role_id": (header >> 56) & 0xF,
        "bits_per_value": (header >> 51) & 0x1F,
        "coarse_bits": (header >> 48) & 0x7,
        "fine_bits": (header >> 45) & 0x7,
        "gain_shift": (header >> 40) & 0x1F,
        "value_count": (header >> 30) & 0x3FF,
        "tile_group_id": (header >> 18) & 0xFFF,
        "reduce_mode": (header >> 15) & 0x7,
        "tail_valid_bits": (header >> 8) & 0x7F,
        "reserved": header & 0xFF,
    }

def fp4llm_sar_code(magnitude, coarse_bits, fine_bits, gain_shift, sign=0):
    scaled = max(float(magnitude), 0.0) / float(1 << max(int(gain_shift), 0))
    if scaled < 0.5:
        return 0

    max_coarse = (1 << int(coarse_bits)) - 1
    mantissa_base = 1 << int(fine_bits)
    max_mantissa = (1 << (int(fine_bits) + 1)) - 1

    coarse = max(int(math.floor(math.log2(scaled))), 0)
    clipped_high = coarse > max_coarse
    coarse = min(coarse, max_coarse)

    step_exp = coarse - int(fine_bits)
    mantissa = int(round(math.ldexp(scaled, -step_exp)))
    mantissa = max(mantissa, mantissa_base)

    if mantissa >= (1 << (int(fine_bits) + 1)):
        if coarse < max_coarse:
            coarse += 1
            mantissa = mantissa_base
        else:
            clipped_high = True

    mantissa = min(mantissa, max_mantissa)
    if clipped_high:
        mantissa = max_mantissa
    fine = mantissa - mantissa_base
    return ((sign & 1) << (int(coarse_bits) + int(fine_bits))) | (coarse << int(fine_bits)) | fine

def rtl_encoder_model(magnitude, coarse_bits, fine_bits, gain_shift):
    return fp4llm_sar_code(magnitude, coarse_bits, fine_bits, gain_shift)

async def csr_cycle(comp, clk, offset, write=False, data=0, timeout=2000):
    comp.wbs_adr_i.value = Force(offset << 2)
    comp.wbs_dat_i.value = Force(data)
    comp.wbs_sel_i.value = Force(0xF)
    comp.wbs_we_i.value = Force(1 if write else 0)
    comp.wbs_cyc_i.value = Force(1)
    comp.wbs_stb_i.value = Force(1)
    value = 0
    for _ in range(timeout):
        await RisingEdge(clk)
        await Timer(1, units="ns")
        if int(comp.wbs_ack_o.value) == 1:
            if not write:
                value = int(comp.wbs_dat_o.value)
            break
    else:
        raise AssertionError(f"Wishbone CSR {'write' if write else 'read'} timeout at offset {offset}")
    comp.wbs_cyc_i.value = Force(0)
    comp.wbs_stb_i.value = Force(0)
    comp.wbs_we_i.value = Force(0)
    await RisingEdge(clk)
    await Timer(1, units="ns")
    return value

async def csr_write(comp, clk, offset, data):
    await csr_cycle(comp, clk, offset, True, data)

async def csr_read(comp, clk, offset):
    return await csr_cycle(comp, clk, offset, False, 0)

async def wait_status(comp, clk, mask=0, timeout_cycles=10000):
    last = 0
    for _ in range(timeout_cycles):
        last = await csr_read(comp, clk, CSR_STATUS)
        if (last & STATUS_INIT) and not (last & STATUS_BUSY) and ((last & mask) == mask):
            return last
    raise AssertionError(f"status timeout: last=0x{last:08x}")

async def start_cmd(comp, clk, command):
    await csr_write(comp, clk, CSR_COMMAND, CMD_START | command)
    return await wait_status(comp, clk, STATUS_RESULT, timeout_cycles=30000)

async def release_forced_wb(comp):
    for sig in (comp.wbs_adr_i, comp.wbs_dat_i, comp.wbs_sel_i, comp.wbs_we_i, comp.wbs_cyc_i, comp.wbs_stb_i):
        sig.value = Release()

async def program_cells(comp, clk, macro, row, cols, data=0xFF):
    for col in cols:
        await csr_write(comp, clk, CSR_ADDR, addr_value(row, col))
        await csr_write(comp, clk, CSR_DATA, data)
        status = await start_cmd(comp, clk, CMD_PROGRAM | cmd_macro(macro))
        assert (status & STATUS_ERROR) == 0, f"program macro={macro} row={row} col={col} status=0x{status:08x}"

async def compute_case(comp, clk, name, row, col, full_row, role, gain, vc, dst, tile, macro_magnitudes):
    bits, coarse, fine = role_policy(role)
    await csr_write(comp, clk, CSR_ROLE, role_value(role, gain, vc, dst, tile))
    await csr_write(comp, clk, CSR_ADDR, addr_value(row, col))
    status = await start_cmd(comp, clk, CMD_COMPUTE | (CMD_FULL_ROW if full_row else 0))
    assert (status & STATUS_ERROR) == 0, f"compute {name} status=0x{status:08x}"
    assert (status & STATUS_NOC) != 0, f"compute {name} did not set NoC valid"

    flit0 = await csr_read(comp, clk, CSR_FLIT0)
    flit1 = await csr_read(comp, clk, CSR_FLIT1)
    flit2 = await csr_read(comp, clk, CSR_FLIT2)
    flit3 = await csr_read(comp, clk, CSR_FLIT3)
    flit = flit0 | (flit1 << 32) | (flit2 << 64) | (flit3 << 96)
    header = flit & ((1 << 64) - 1)
    payload = (flit >> 64) & ((1 << 64) - 1)
    fields = decode_header(header)
    values = unpack_lsb(payload, len(macro_magnitudes), bits)

    expected_rtl = [rtl_encoder_model(v, coarse, fine, gain) for v in macro_magnitudes]
    expected_fp4llm = [fp4llm_sar_code(v, coarse, fine, gain) for v in macro_magnitudes]

    assert fields["fmt_id"] == 1
    assert fields["role_id"] == role
    assert fields["bits_per_value"] == bits
    assert fields["coarse_bits"] == coarse
    assert fields["fine_bits"] == fine
    assert fields["gain_shift"] == gain
    assert fields["value_count"] == len(macro_magnitudes)
    assert fields["tile_group_id"] == tile
    assert fields["reduce_mode"] == 1
    assert fields["tail_valid_bits"] == tail_valid_bits(64 + len(macro_magnitudes) * bits)
    assert values == expected_fp4llm, f"{name}: FP4LLM SAR-FP mismatch got={values} expected={expected_fp4llm}"

    row_report = {
        "case": name,
        "role_id": role,
        "gain_shift": gain,
        "macro_magnitudes": list(macro_magnitudes),
        "header": fields,
        "actual_payload_values": values,
        "rtl_encoder_expected_values": expected_rtl,
        "fp4llm_expected_values": expected_fp4llm,
        "matches_current_rtl_encoder": values == expected_rtl,
        "matches_fp4llm_sar_fp_encoder": values == expected_fp4llm,
        "fp4llm_default_values_per_tile": FP4LLM_VALUE_COUNT,
        "fp4llm_default_aggregate_tiles": FP4LLM_AGGREGATE_TILES,
        "fp4llm_default_values_per_packet": FP4LLM_VALUE_COUNT * FP4LLM_AGGREGATE_TILES,
    }
    cocotb.log.info(
        f"[FP4LLM_COMPARE] {name}: actual={values} fp4llm={expected_fp4llm} "
        f"rtl_model={expected_rtl} match_fp4llm={row_report['matches_fp4llm_sar_fp_encoder']}"
    )
    return row_report

@cocotb.test()
@report_test
async def x1_memory_compiler_noc(dut):
    caravelEnv = await test_configure(dut, timeout_cycles=500_000)
    cocotb.log.info("[TEST] Start x1_memory_compiler_noc direct CSR smoke")

    comp = dut.uut.chip_core.mprj.x1_mem_compiler
    clk = caravelEnv.clk

    comp.wbs_cyc_i.value = Force(0)
    comp.wbs_stb_i.value = Force(0)
    comp.wbs_we_i.value = Force(0)
    comp.wbs_sel_i.value = Force(0xF)
    await ClockCycles(clk, 5)

    await wait_status(comp, clk, 0)

    await csr_write(comp, clk, CSR_ADDR, addr_value(8, 9))
    await csr_write(comp, clk, CSR_DATA, 0xFF)
    status = await start_cmd(comp, clk, CMD_PROGRAM | cmd_macro(0))
    assert (status & STATUS_ERROR) == 0
    status = await start_cmd(comp, clk, CMD_PROGRAM | cmd_macro(1))
    assert (status & STATUS_ERROR) == 0

    status = await start_cmd(comp, clk, CMD_READ | cmd_macro(0))
    assert (status & STATUS_ERROR) == 0
    result = await csr_read(comp, clk, CSR_RESULT)
    assert (result & 0x1) == 1

    await csr_write(comp, clk, CSR_ROLE, role_value(2, 0, 1, 3, 0x155))
    status = await start_cmd(comp, clk, CMD_COMPUTE)
    assert (status & STATUS_NOC) != 0
    assert int(comp.noc_flit_valid.value) == 1
    assert int(comp.noc_flit_last.value) == 1
    assert int(comp.noc_flit_vc.value) == 1
    assert int(comp.noc_dst.value) == 3

    flit0 = await csr_read(comp, clk, CSR_FLIT0)
    flit1 = await csr_read(comp, clk, CSR_FLIT1)
    flit2 = await csr_read(comp, clk, CSR_FLIT2)
    flit3 = await csr_read(comp, clk, CSR_FLIT3)
    flit = flit0 | (flit1 << 32) | (flit2 << 64) | (flit3 << 96)
    header = flit & ((1 << 64) - 1)
    payload = (flit >> 64) & ((1 << 64) - 1)

    assert ((header >> 60) & 0xF) == 1
    assert ((header >> 56) & 0xF) == 2
    assert ((header >> 51) & 0x1F) == 9
    assert ((header >> 48) & 0x7) == 5
    assert ((header >> 45) & 0x7) == 3
    assert ((header >> 30) & 0x3FF) == 4
    assert ((header >> 18) & 0xFFF) == 0x155
    assert ((header >> 15) & 0x7) == 1
    assert ((header >> 8) & 0x7F) == 100

    values = unpack_lsb(payload, 4, 9)
    assert values == [0, 0, 0, 0]
    cocotb.log.info(f"[TEST] X1 NoC header=0x{header:016x} payload_values={values}")

    report = []
    report.append(await compute_case(comp, clk, "single_cell_role2_gain0", 8, 9, False, 2, 0, 1, 3, 0x155, [1, 1, 0, 0]))

    await program_cells(comp, clk, macro=0, row=3, cols=range(8), data=0xFF)
    report.append(await compute_case(comp, clk, "eight_hits_role1_gain0", 3, 0, True, 1, 0, 1, 3, 0x201, [8, 0, 0, 0]))
    report.append(await compute_case(comp, clk, "eight_hits_role2_gain0", 3, 0, True, 2, 0, 1, 3, 0x202, [8, 0, 0, 0]))
    report.append(await compute_case(comp, clk, "eight_hits_role8_gain0", 3, 0, True, 8, 0, 1, 3, 0x208, [8, 0, 0, 0]))
    report.append(await compute_case(comp, clk, "eight_hits_role2_gain2", 3, 0, True, 2, 2, 1, 3, 0x222, [8, 0, 0, 0]))

    mismatches = [item for item in report if not item["matches_fp4llm_sar_fp_encoder"]]
    summary = {
        "summary": {
            "cases": len(report),
            "fp4llm_mismatch_cases": len(mismatches),
            "observed_value_count": 4,
            "fp4llm_default_values_per_packet": FP4LLM_VALUE_COUNT * FP4LLM_AGGREGATE_TILES,
        },
        "cases": report,
    }
    report_path = Path.cwd() / "fp4llm_compare_report.json"
    report_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    cocotb.log.info(f"[FP4LLM_COMPARE] wrote {report_path}")
    cocotb.log.info(f"[FP4LLM_COMPARE] fp4llm_mismatch_cases={len(mismatches)}/{len(report)}")

    await release_forced_wb(comp)
