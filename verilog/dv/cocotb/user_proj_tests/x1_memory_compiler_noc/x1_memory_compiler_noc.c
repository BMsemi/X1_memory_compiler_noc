#define USER_ADDR_SPACE_C_HEADER_FILE

#include <firmware_apis.h>
#include <custom_user_space.h>
#include <stdint.h>

#define CSR_VERSION 0u
#define CSR_CONFIG  1u
#define CSR_STATUS  2u
#define CSR_COMMAND 3u
#define CSR_ADDR    4u
#define CSR_DATA    5u
#define CSR_ROLE    6u
#define CSR_RESULT  7u
#define CSR_FLIT0   8u
#define CSR_FLIT1   9u
#define CSR_FLIT2   10u
#define CSR_FLIT3   11u

#define STATUS_BUSY   0x01u
#define STATUS_INIT   0x02u
#define STATUS_RESULT 0x04u
#define STATUS_NOC    0x08u
#define STATUS_ERROR  0x10u

#define CMD_START   0x80000000u
#define CMD_PROGRAM 0x1u
#define CMD_RESET   0x2u
#define CMD_READ    0x3u
#define CMD_COMPUTE 0x4u
#define CMD_MACRO(m) (((uint32_t)(m) & 0xFu) << 4)
#define CMD_FULL_ROW (1u << 9)

#define ROLE_FFN_DOWN 2u
#define ROLE_VALUE(role, gain, vc, dst, tile) \
    ((((uint32_t)(tile) & 0xFFFu) << 16) | (((uint32_t)(dst) & 0xFu) << 12) | \
     (((uint32_t)(vc) & 0x7u) << 9) | (((uint32_t)(gain) & 0x1Fu) << 4) | ((uint32_t)(role) & 0xFu))
#define ADDR_VALUE(row, col) ((((uint32_t)(col) & 0x1Fu) << 8) | ((uint32_t)(row) & 0x1Fu))

static inline void wait_cycles(uint32_t cycles)
{
    for (uint32_t i = 0; i < cycles; i++) {
        __asm__ volatile ("nop");
    }
}

static uint32_t wait_status(uint32_t mask)
{
    uint32_t status = 0;
    for (uint32_t i = 0; i < 300000u; i++) {
        status = (uint32_t)USER_readWord(CSR_STATUS);
        if ((status & STATUS_INIT) && !(status & STATUS_BUSY) && ((status & mask) == mask)) {
            return status;
        }
        wait_cycles(4);
    }
    return status | STATUS_ERROR;
}

static uint32_t run_cmd(uint32_t cmd)
{
    USER_writeWord((int)(CMD_START | cmd), CSR_COMMAND);
    return wait_status(STATUS_RESULT);
}

void main()
{
    uint32_t ok = 1u;

    ManagmentGpio_outputEnable();
    ManagmentGpio_write(0);
    enableHkSpi(0);
    User_enableIF(1);

    if (wait_status(0) & STATUS_ERROR) ok = 0u;
    if ((uint32_t)USER_readWord(CSR_VERSION) != 0x58314E43u) ok = 0u;

    USER_writeWord((int)ADDR_VALUE(8, 9), CSR_ADDR);
    USER_writeWord(0xFF, CSR_DATA);
    if (run_cmd(CMD_PROGRAM | CMD_MACRO(0)) & STATUS_ERROR) ok = 0u;
    if (run_cmd(CMD_PROGRAM | CMD_MACRO(1)) & STATUS_ERROR) ok = 0u;

    USER_writeWord((int)ADDR_VALUE(8, 9), CSR_ADDR);
    if (run_cmd(CMD_READ | CMD_MACRO(0)) & STATUS_ERROR) ok = 0u;
    if (((uint32_t)USER_readWord(CSR_RESULT) & 1u) != 1u) ok = 0u;

    USER_writeWord((int)ROLE_VALUE(ROLE_FFN_DOWN, 0, 1, 3, 0x155), CSR_ROLE);
    USER_writeWord((int)ADDR_VALUE(8, 9), CSR_ADDR);
    if (run_cmd(CMD_COMPUTE) & STATUS_ERROR) ok = 0u;

    uint32_t status = (uint32_t)USER_readWord(CSR_STATUS);
    uint32_t header_lo = (uint32_t)USER_readWord(CSR_FLIT0);
    uint32_t header_hi = (uint32_t)USER_readWord(CSR_FLIT1);
    uint32_t payload_lo = (uint32_t)USER_readWord(CSR_FLIT2);

    uint32_t fmt_id = (header_hi >> 28) & 0xFu;
    uint32_t role_id = (header_hi >> 24) & 0xFu;
    uint32_t bits_per_value = (header_hi >> 19) & 0x1Fu;
    uint32_t coarse_bits = (header_hi >> 16) & 0x7u;
    uint32_t fine_bits = (header_hi >> 13) & 0x7u;
    uint32_t value_count = ((header_hi & 0xFFu) << 2) | ((header_lo >> 30) & 0x3u);
    uint32_t tile_group = (header_lo >> 18) & 0xFFFu;
    uint32_t reduce_mode = (header_lo >> 15) & 0x7u;
    uint32_t tail_valid = (header_lo >> 8) & 0x7Fu;

    if ((status & STATUS_NOC) == 0u) ok = 0u;
    if (fmt_id != 1u || role_id != ROLE_FFN_DOWN || bits_per_value != 9u) ok = 0u;
    if (coarse_bits != 5u || fine_bits != 3u || value_count != 4u) ok = 0u;
    if (tile_group != 0x155u || reduce_mode != 1u || tail_valid != 100u) ok = 0u;
    if (payload_lo != 0u) ok = 0u;

    ManagmentGpio_write(ok ? 1 : 0);
    while (1) { }
}
