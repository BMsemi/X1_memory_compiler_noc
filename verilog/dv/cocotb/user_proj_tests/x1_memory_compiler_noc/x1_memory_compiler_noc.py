from caravel_cocotb.caravel_interfaces import test_configure
from caravel_cocotb.caravel_interfaces import report_test
import cocotb
from cocotb.triggers import RisingEdge, ClockCycles, Timer
from cocotb.handle import Force, Release

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

def cmd_macro(idx):
    return (idx & 0xF) << 4

def addr_value(row, col):
    return ((col & 0x1F) << 8) | (row & 0x1F)

def role_value(role, gain, vc, dst, tile):
    return ((tile & 0xFFF) << 16) | ((dst & 0xF) << 12) | ((vc & 0x7) << 9) | ((gain & 0x1F) << 4) | (role & 0xF)

def unpack_lsb(payload, count, bits):
    mask = (1 << bits) - 1
    return [(payload >> (idx * bits)) & mask for idx in range(count)]

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
    assert values == [4, 4, 0, 0]
    await release_forced_wb(comp)
    cocotb.log.info(f"[TEST] X1 NoC header=0x{header:016x} payload_values={values}")
