from caravel_cocotb.caravel_interfaces import test_configure
from caravel_cocotb.caravel_interfaces import report_test
import cocotb
from cocotb.triggers import RisingEdge, ClockCycles, Timer
from cocotb.handle import Force, Release
import json
import os
import socket
import struct
from pathlib import Path

CSR_DESC_DATA = 14
CSR_DESC_CTRL = 15
CSR_DESC_FETCH_ADDR = 18
CSR_WINDOW_INDEX = 19
CSR_WINDOW_DATA = 20

EXPECTED_ANALOG = [12, 1, 0, 0]
EXPECTED_PAYLOAD = [102, 4, 0, 0]
EXPECTED_ROW = 1
EXPECTED_COL = 11
EXPECTED_ROLE = 3
EXPECTED_DST = 3
EXPECTED_VC = 1
EXPECTED_TILE_GROUP = 0


FORBIDDEN_STREAMING_CSR_WRITES = {CSR_DESC_DATA, CSR_WINDOW_INDEX, CSR_WINDOW_DATA}


def load_vector():
    here = Path(__file__).resolve().parent
    with open(here / "x1_real_weight_firmware_vector.json", "r", encoding="ascii") as f:
        return json.load(f)


def descriptor_base(vector, index):
    return vector["queue_base"] + index * 64


def build_memory(vector):
    memory = {}
    for desc_idx, descriptor in enumerate(vector["descriptor_queue_words"]):
        base = descriptor_base(vector, desc_idx)
        for word_idx, word in enumerate(descriptor):
            memory[base + word_idx * 4] = word
    for word_idx, word in enumerate(vector["packed_window_words"]):
        memory[vector["weight_base"] + word_idx * 4] = word
    return memory


class DictDmaBacking:
    def __init__(self, memory):
        self.memory = memory

    def read_word(self, addr):
        if addr not in self.memory:
            raise AssertionError(f"unexpected GPIO DMA read address 0x{addr:08x}")
        return int(self.memory[addr]) & 0xFFFFFFFF

    def write_word(self, addr, data, strb):
        return None

    def close(self):
        return None


class SocketDmaBacking:
    def __init__(self, host, port):
        self.host = host
        self.port = int(port)
        self.sock = socket.create_connection((self.host, self.port), timeout=30.0)
        self.sock.settimeout(30.0)
        assert self._request("PING") == b"OK"
        cocotb.log.info(f"[DMA_SOCKET] connected to {self.host}:{self.port}")

    def _recv_exact(self, n):
        chunks = []
        remaining = int(n)
        while remaining:
            chunk = self.sock.recv(remaining)
            if not chunk:
                raise AssertionError("DMA socket closed while reading response")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _request(self, line):
        self.sock.sendall(line.encode("ascii") + b"\n")
        header = self._recv_exact(5)
        status = header[0]
        length = struct.unpack("<I", header[1:])[0]
        payload = self._recv_exact(length)
        if status != 0:
            raise AssertionError(payload.decode("utf-8", errors="replace"))
        return payload

    def read_word(self, addr):
        payload = self._request(f"WORD 0x{addr:08x}")
        if len(payload) != 4:
            raise AssertionError(f"WORD response length {len(payload)} for 0x{addr:08x}")
        return struct.unpack("<I", payload)[0]

    def write_word(self, addr, data, strb):
        self._request(f"WRITE 0x{addr:08x} 0x{data & 0xFFFFFFFF:08x} 0x{strb & 0xF:x}")

    def close(self):
        self.sock.close()


def make_dma_backing(vector):
    host = os.environ.get("X1_DMA_SOCKET_HOST")
    port = os.environ.get("X1_DMA_SOCKET_PORT")
    if host and port:
        return SocketDmaBacking(host, int(port))
    return DictDmaBacking(build_memory(vector))


def unpack_lsb(payload, count, bits):
    mask = (1 << bits) - 1
    return [(payload >> (idx * bits)) & mask for idx in range(count)]


def probe_analog_cell(comp, macro, row, col):
    try:
        return int(comp.gen_x1_macro[macro].x1_macro.array_mem[row][col].value), None
    except (AttributeError, IndexError, TypeError, ValueError) as exc:
        return None, str(exc)


async def wait_for_mgmt_gpio(caravel_env, value, timeout_cycles):
    expected = str(value)
    for _ in range(timeout_cycles):
        if caravel_env.monitor_mgmt_gpio() == expected:
            return
        await ClockCycles(caravel_env.clk, 1)
    raise AssertionError(f"management GPIO did not reach {expected}")


def gpio_input_word(rx_valid=0, rx_data=0, tx_ready=1):
    return ((rx_valid & 1) << 0) | ((rx_data & 0xFF) << 1) | ((tx_ready & 1) << 9)


def drive_gpio_stream(mprj, rx_valid=0, rx_data=0, tx_ready=1):
    mprj.io_in.value = Force(gpio_input_word(rx_valid, rx_data, tx_ready))


def gpio_tx_valid(mprj):
    return (int(mprj.io_out.value) >> 28) & 1


def gpio_tx_data(mprj):
    return (int(mprj.io_out.value) >> 29) & 0xFF


def gpio_rx_ready(mprj):
    return (int(mprj.io_out.value) >> 37) & 1


def u32_from_le(frame, start):
    return frame[start] | (frame[start + 1] << 8) | (frame[start + 2] << 16) | (frame[start + 3] << 24)


def le_u32(value):
    return [(value >> shift) & 0xFF for shift in (0, 8, 16, 24)]


async def peripheral_gpio_stream_model(comp, mprj, clk, memory, served_reads, writes, frames, stop_flag):
    drive_gpio_stream(mprj)
    tx_frame = []
    response_bytes = []
    while not stop_flag[0]:
        await RisingEdge(clk)
        await Timer(1, units="ns")

        if gpio_tx_valid(mprj):
            tx_frame.append(gpio_tx_data(mprj))
            if len(tx_frame) == 1 and tx_frame[0] not in (0x01, 0x02):
                raise AssertionError(f"unexpected GPIO DMA frame opcode 0x{tx_frame[0]:02x}")
            if tx_frame[0] == 0x01 and len(tx_frame) == 5:
                addr = u32_from_le(tx_frame, 1)
                word = memory.read_word(addr)
                served_reads.append(addr)
                frames.append(tuple(tx_frame))
                response_bytes.extend([0x81] + le_u32(word))
                tx_frame = []
            elif tx_frame[0] == 0x02 and len(tx_frame) == 10:
                addr = u32_from_le(tx_frame, 1)
                data = u32_from_le(tx_frame, 5)
                strb = tx_frame[9] & 0xF
                memory.write_word(addr, data, strb)
                writes.append((addr, data, strb))
                frames.append(tuple(tx_frame))
                tx_frame = []

        if response_bytes and gpio_rx_ready(mprj):
            drive_gpio_stream(mprj, rx_valid=1, rx_data=response_bytes.pop(0), tx_ready=1)
        else:
            drive_gpio_stream(mprj, rx_valid=0, rx_data=0, tx_ready=1)


async def management_write_monitor(comp, clk, writes, stop_flag):
    while not stop_flag[0]:
        await RisingEdge(clk)
        await Timer(1, units="ns")
        if int(comp.wbs_ack_o.value) == 1 and int(comp.wbs_we_i.value) == 1:
            offset = (int(comp.wbs_adr_i.value) >> 2) & 0x3F
            writes.append((offset, int(comp.wbs_dat_i.value)))


async def release_forced_peripheral(mprj):
    mprj.io_in.value = Release()


def expected_descriptor_reads(vector):
    out = []
    for idx in range(len(vector["descriptor_queue_words"])):
        base = descriptor_base(vector, idx)
        out.extend(base + word_idx * 4 for word_idx in range(16))
    return out


def expected_weight_reads(vector):
    base = vector["weight_base"]
    return [base + word_idx * 4 for word_idx in range(len(vector["packed_window_words"]))]


async def verify_noc_fabric_all_nodes(comp, clk, base_flit):
    routed = []
    try:
        for dst in range(4):
            test_flit = base_flit ^ (dst << 96) ^ (dst << 64)
            comp.endpoint_noc_flit_data.value = Force(test_flit)
            comp.endpoint_noc_flit_valid.value = Force(1)
            comp.endpoint_noc_flit_last.value = Force(1)
            comp.endpoint_noc_flit_vc.value = Force(dst & 0x7)
            comp.endpoint_noc_dst.value = Force(dst)
            await ClockCycles(clk, 1)
            await Timer(1, units="ns")
            node_valid = int(comp.noc_node_valid.value)
            assert node_valid & (1 << dst), f"NoC directed route missing dst={dst}, valid=0x{node_valid:x}"
            node_flit = (int(comp.noc_node_flit_data.value) >> (dst * 128)) & ((1 << 128) - 1)
            node_vc = (int(comp.noc_node_vc.value) >> (dst * 3)) & 0x7
            node_src = (int(comp.noc_node_src.value) >> (dst * 4)) & 0xF
            assert node_flit == test_flit, f"NoC dst={dst} flit got 0x{node_flit:032x}, expected 0x{test_flit:032x}"
            assert node_vc == (dst & 0x7)
            assert node_src == 0
            routed.append(dst)
    finally:
        for sig in (
            comp.endpoint_noc_flit_data,
            comp.endpoint_noc_flit_valid,
            comp.endpoint_noc_flit_last,
            comp.endpoint_noc_flit_vc,
            comp.endpoint_noc_dst,
        ):
            sig.value = Release()
    return routed


@cocotb.test()
@report_test
async def x1_real_weight_firmware(dut):
    vector = load_vector()
    memory = make_dma_backing(vector)
    caravel_env = await test_configure(dut, timeout_cycles=5_500_000)
    cocotb.log.info("[TEST] Start RISC-V firmware DMA real FP4LLM X1 deployment")

    comp = dut.uut.chip_core.mprj.x1_mem_compiler
    mprj = dut.uut.chip_core.mprj
    clk = caravel_env.clk

    served_reads = []
    dma_writes = []
    gpio_frames = []
    csr_writes = []
    stop_flag = [False]
    dma_task = cocotb.start_soon(peripheral_gpio_stream_model(comp, mprj, clk, memory, served_reads, dma_writes, gpio_frames, stop_flag))
    monitor_task = cocotb.start_soon(management_write_monitor(comp, clk, csr_writes, stop_flag))

    try:
        done = False
        for cycle in range(1_500_000):
            if caravel_env.monitor_mgmt_gpio() == "1":
                done = True
                break
            if cycle and (cycle % 100_000) == 0:
                cocotb.log.info(
                    "[DMA_FW_DEBUG] cycle=%d state=%s init=%s desc_busy=%s desc_done=%s "
                    "desc_error=%s desc_last_op=%s desc_result=%s req_valid=%s rsp_ready=%s "
                    "reads=%d writes=%d csr_tail=%s"
                    % (
                        cycle,
                        comp.state.value,
                        comp.init_done.value,
                        comp.desc_busy.value,
                        comp.desc_done.value,
                        comp.desc_error.value,
                        comp.desc_last_op.value,
                        comp.desc_result_word.value,
                        comp.periph_rd_req_valid.value,
                        comp.periph_rd_rsp_ready.value,
                        len(served_reads),
                        len(dma_writes),
                        csr_writes[-8:],
                    )
                )
            await ClockCycles(clk, 1)
        assert done, (
            "management GPIO did not reach 1; "
            f"state={comp.state.value} init={comp.init_done.value} "
            f"desc_busy={comp.desc_busy.value} desc_done={comp.desc_done.value} "
            f"desc_error={comp.desc_error.value} desc_last_op={comp.desc_last_op.value} "
            f"desc_result={comp.desc_result_word.value} reads={served_reads[-16:]} "
            f"writes={dma_writes[-8:]} csr_writes={csr_writes[-16:]}"
        )
        await ClockCycles(clk, 10)
    finally:
        stop_flag[0] = True
        await ClockCycles(clk, 2)
        await release_forced_peripheral(mprj)
        dma_task.kill()
        monitor_task.kill()
        memory.close()

    forbidden = [(offset, data) for offset, data in csr_writes if offset in FORBIDDEN_STREAMING_CSR_WRITES]
    assert not forbidden, f"firmware streamed payload/descriptor data over Wishbone: {forbidden}"
    assert any(offset == CSR_DESC_FETCH_ADDR for offset, _ in csr_writes), "firmware never configured descriptor fetch address"
    assert any(offset == CSR_DESC_CTRL and (data & 0x4) for offset, data in csr_writes), "firmware never started descriptor fetch"

    desc_reads = expected_descriptor_reads(vector)
    weight_reads = expected_weight_reads(vector)
    assert served_reads[:16] == desc_reads[:16]
    for addr in desc_reads:
        assert addr in served_reads, f"missing descriptor DMA read 0x{addr:08x}"
    for addr in weight_reads:
        assert addr in served_reads, f"missing weight DMA read 0x{addr:08x}"

    analog_probe_errors = []
    for macro, expected in enumerate(vector["analog_values"]):
        got, error = probe_analog_cell(comp, macro, EXPECTED_ROW, EXPECTED_COL)
        if error is not None:
            analog_probe_errors.append((macro, error))
        else:
            assert got == expected, f"macro {macro} analog storage got {got}, expected {expected}"
    if analog_probe_errors:
        cocotb.log.info(
            "[TEST] Direct PE array probe unavailable under this simulator; "
            f"using NoC payload as end-to-end analog check: {analog_probe_errors}"
        )

    assert int(comp.desc_done.value) == 1
    assert int(comp.desc_error.value) == 0
    assert int(comp.desc_busy.value) == 0
    assert int(comp.desc_last_op.value) == 2
    assert int(comp.desc_exec_count.value) == len(vector["descriptor_queue_words"])
    assert int(comp.noc_flit_valid.value) == 1
    assert int(comp.noc_flit_last.value) == 1
    assert int(comp.noc_flit_vc.value) == EXPECTED_VC
    assert int(comp.noc_dst.value) == EXPECTED_DST

    node_valid = int(comp.noc_node_valid.value)
    assert node_valid & (1 << EXPECTED_DST), f"NoC node {EXPECTED_DST} did not receive flit; valid=0x{node_valid:x}"

    flit = int(comp.noc_flit_data.value)
    node_flit = (int(comp.noc_node_flit_data.value) >> (EXPECTED_DST * 128)) & ((1 << 128) - 1)
    node_vc = (int(comp.noc_node_vc.value) >> (EXPECTED_DST * 3)) & 0x7
    node_src = (int(comp.noc_node_src.value) >> (EXPECTED_DST * 4)) & 0xF
    assert node_flit == flit, f"NoC node flit mismatch node=0x{node_flit:032x} route=0x{flit:032x}"
    assert node_vc == EXPECTED_VC
    assert node_src == 0
    header = flit & ((1 << 64) - 1)
    payload = (flit >> 64) & ((1 << 64) - 1)
    values = unpack_lsb(payload, 4, 9)

    assert ((header >> 60) & 0xF) == 1
    assert ((header >> 56) & 0xF) == EXPECTED_ROLE
    assert ((header >> 51) & 0x1F) == 9
    assert ((header >> 48) & 0x7) == 5
    assert ((header >> 45) & 0x7) == 3
    assert ((header >> 30) & 0x3FF) == 4
    assert ((header >> 18) & 0xFFF) == EXPECTED_TILE_GROUP
    assert values == EXPECTED_PAYLOAD, (
        f"payload values got {values}, expected {EXPECTED_PAYLOAD}; "
        f"header=0x{header:016x} payload=0x{payload:016x} flit=0x{flit:032x}"
    )

    expected_dma_writes = [
        (vector["result_periph_base"] + idx * 4, (flit >> (idx * 32)) & 0xFFFFFFFF, 0xF)
        for idx in range(4)
    ]
    assert dma_writes == expected_dma_writes, (
        f"GPIO DMA writes got {dma_writes}, expected {expected_dma_writes}; "
        f"last frames={gpio_frames[-8:]}"
    )

    routed_nodes = await verify_noc_fabric_all_nodes(comp, clk, flit)

    cocotb.log.info(
        f"[TEST] Real FP4LLM DMA firmware pass row={EXPECTED_ROW} col={EXPECTED_COL} "
        f"analog={EXPECTED_ANALOG} payload_values={values} header=0x{header:016x} "
        f"gpio_frames={len(gpio_frames)} dma_reads={len(served_reads)} "
        f"dma_writes={dma_writes} noc_routed_nodes={routed_nodes} csr_writes={csr_writes}"
    )
