#define USER_ADDR_SPACE_C_HEADER_FILE

#include <firmware_apis.h>
#include <custom_user_space.h>
#include <stdint.h>
#include "x1_real_weight_firmware_vector.h"

#define CSR_VERSION 0u
#define CSR_CONFIG  1u
#define CSR_STATUS  2u
#define CSR_DESC_CTRL   15u
#define CSR_DESC_STATUS 16u
#define CSR_DESC_RESULT 17u
#define CSR_DESC_FETCH_ADDR 18u

#define STATUS_BUSY   0x01u
#define STATUS_INIT   0x02u
#define STATUS_ERROR  0x10u

#define DESC_DONE        0x02u
#define DESC_ERROR       0x04u
#define DESC_CHECKSUM_OK 0x08u

#define DESC_CTRL_CLEAR       0x02u
#define DESC_CTRL_START_FETCH 0x04u

static inline void wait_cycles(uint32_t cycles)
{
    for (uint32_t i = 0u; i < cycles; i++) {
        __asm__ volatile ("nop");
    }
}

static void fail(void)
{
    ManagmentGpio_write(0);
    while (1) { }
}

static void require_true(uint32_t condition)
{
    if (!condition) {
        fail();
    }
}

static uint32_t wait_init(void)
{
    uint32_t status = 0u;
    for (uint32_t i = 0u; i < 1000000u; i++) {
        status = USER_readWord(CSR_STATUS);
        if ((status & STATUS_INIT) && !(status & STATUS_BUSY)) {
            return status;
        }
        wait_cycles(4u);
    }
    return status | STATUS_ERROR;
}

static uint32_t wait_descriptor_done(void)
{
    uint32_t status = 0u;
    for (uint32_t i = 0u; i < 3000000u; i++) {
        status = USER_readWord(CSR_DESC_STATUS);
        if ((status & DESC_DONE) != 0u) {
            return status;
        }
        wait_cycles(4u);
    }
    return status | DESC_ERROR;
}

static uint32_t descriptor_addr(uint32_t index)
{
    return X1_REAL_QUEUE_BASE + (index * X1_REAL_DESCRIPTOR_WORDS * 4u);
}

static uint32_t run_fetched_descriptor(uint32_t index, uint32_t expected_op)
{
    USER_writeWord(DESC_CTRL_CLEAR, CSR_DESC_CTRL);
    USER_writeWord(descriptor_addr(index), CSR_DESC_FETCH_ADDR);
    USER_writeWord(DESC_CTRL_START_FETCH, CSR_DESC_CTRL);

    uint32_t status = wait_descriptor_done();
    require_true((status & DESC_ERROR) == 0u);
    require_true((status & DESC_CHECKSUM_OK) != 0u);
    require_true(((status >> 8) & 0xFFu) == expected_op);
    return USER_readWord(CSR_DESC_RESULT);
}

void main()
{
    ManagmentGpio_outputEnable();
    ManagmentGpio_write(0);
    enableHkSpi(0);
    User_enableIF();

    require_true((wait_init() & STATUS_ERROR) == 0u);
    require_true(USER_readWord(CSR_VERSION) == 0x58314E43u);
    require_true(((USER_readWord(CSR_CONFIG) >> 16) & 0xFFu) == X1_REAL_NUM_MACROS);

    for (uint32_t i = 0u; i < X1_REAL_DESCRIPTOR_COUNT; i++) {
        uint32_t result = run_fetched_descriptor(i, x1_real_expected_ops[i]);
        if (x1_real_expected_ops[i] == X1_REAL_OP_EXECUTE_TILE) {
            require_true((result & 0xFFFu) == x1_real_expected_results[i]);
        } else {
            require_true((result & 0xFFFFu) == x1_real_expected_results[i]);
        }
    }

    ManagmentGpio_write(1);
    while (1) { }
}
