`default_nettype none

// X1-only memory compiler and SAR-TDC NoC endpoint.
// This module intentionally instantiates only the submit behavioral X1 macro.
module x1_memory_compiler_noc #(
    parameter integer NUM_MACROS = 4,
    parameter integer X1_WRITE_DONE_CYCLES = 512,
    parameter integer X1_READ_DONE_CYCLES = 256,
    parameter integer X1_INIT_DRAIN_CYCLES = 512
) (
`ifdef USE_POWER_PINS
    inout         VDDC1,
    inout         VDDC2,
    inout         VDDA1,
    inout         VDDA2,
    inout         VSS,
`endif
    input         wb_clk_i,
    input         wb_rst_i,
    input         wbs_stb_i,
    input         wbs_cyc_i,
    input         wbs_we_i,
    input  [3:0]  wbs_sel_i,
    input  [31:0] wbs_dat_i,
    input  [31:0] wbs_adr_i,
    output reg    wbs_ack_o,
    output reg [31:0] wbs_dat_o,

    input         ScanInCC,
    input         ScanInDL,
    input         ScanInDR,
    input         TM,
    output        ScanOutCC,

    input         Iref,
    input         Vcc_read,
    input         Vcomp,
    input         Bias_comp2,
    input         Vcc_wl_read,
    input         Vcc_wl_set,
    input         Vbias,
    input         Vcc_wl_reset,
    input         Vcc_set,
    input         dc_bias,

    output reg [127:0] noc_flit_data,
    output reg         noc_flit_valid,
    output reg         noc_flit_last,
    output reg [2:0]   noc_flit_vc,
    output reg [3:0]   noc_dst,
    output             compiler_busy,
    output reg         init_done,
    output reg         result_valid,
    output reg         error_sticky
);

    localparam [5:0] REG_VERSION = 6'd0;
    localparam [5:0] REG_CONFIG  = 6'd1;
    localparam [5:0] REG_STATUS  = 6'd2;
    localparam [5:0] REG_COMMAND = 6'd3;
    localparam [5:0] REG_ADDR    = 6'd4;
    localparam [5:0] REG_DATA    = 6'd5;
    localparam [5:0] REG_ROLE    = 6'd6;
    localparam [5:0] REG_RESULT  = 6'd7;
    localparam [5:0] REG_FLIT0   = 6'd8;
    localparam [5:0] REG_FLIT1   = 6'd9;
    localparam [5:0] REG_FLIT2   = 6'd10;
    localparam [5:0] REG_FLIT3   = 6'd11;
    localparam [5:0] REG_CLEAR   = 6'd12;

    localparam [3:0] CMD_NOP     = 4'd0;
    localparam [3:0] CMD_PROGRAM = 4'd1;
    localparam [3:0] CMD_RESET   = 4'd2;
    localparam [3:0] CMD_READ    = 4'd3;
    localparam [3:0] CMD_COMPUTE = 4'd4;

    localparam [1:0] X1_MODE_RESET   = 2'b00;
    localparam [1:0] X1_MODE_READ    = 2'b01;
    localparam [1:0] X1_MODE_COMPUTE = 2'b10;
    localparam [1:0] X1_MODE_PROGRAM = 2'b11;

    localparam [3:0] FMT_SAR_FP = 4'd1;
    localparam [2:0] REDUCE_LOCAL_ROW = 3'd1;

    localparam [4:0] S_INIT_ISSUE      = 5'd0;
    localparam [4:0] S_WRITE_ISSUE     = 5'd1;
    localparam [4:0] S_WAIT_WRITE_ACK  = 5'd2;
    localparam [4:0] S_DELAY           = 5'd3;
    localparam [4:0] S_IDLE            = 5'd4;
    localparam [4:0] S_DISPATCH        = 5'd5;
    localparam [4:0] S_READ_POP_ISSUE  = 5'd6;
    localparam [4:0] S_WAIT_READ_ACK   = 5'd7;
    localparam [4:0] S_COMPUTE_PREP    = 5'd8;
    localparam [4:0] S_DONE            = 5'd9;
    localparam [4:0] S_CAPTURE_READ    = 5'd10;

    localparam [2:0] CTX_INIT          = 3'd0;
    localparam [2:0] CTX_PROGRAM_DONE  = 3'd1;
    localparam [2:0] CTX_READ_CMD      = 3'd2;
    localparam [2:0] CTX_READ_POP      = 3'd3;
    localparam [2:0] CTX_COMPUTE_WRITE = 3'd4;
    localparam [2:0] CTX_COMPUTE_POP   = 3'd5;
    localparam [2:0] CTX_INIT_DONE     = 3'd6;

    localparam [31:0] CFG_WORD0 = 32'hA203_C40F;
    localparam [31:0] CFG_WORD1 = 32'h0F03_0D43;
    localparam [31:0] CFG_WORD2 = {2'b01, 3'b000, 7'd32, 10'd3, 10'd3};

    reg [4:0] state;
    reg [2:0] x1_context;
    reg [2:0] delay_context;
    reg [15:0] wait_counter;
    reg [7:0] retry_counter;

    reg [4:0] row_reg;
    reg [4:0] col_reg;
    reg [7:0] data_reg;
    reg [3:0] role_reg;
    reg [4:0] gain_shift_reg;
    reg [2:0] vc_reg;
    reg [3:0] dst_reg;
    reg [11:0] tile_group_reg;
    reg [3:0] user_op;
    reg [7:0] user_macro;
    reg       user_full_row;

    reg [7:0] init_macro_idx;
    reg [1:0] init_pkt_idx;
    reg [7:0] active_macro;
    reg [7:0] compute_macro_idx;
    reg [1:0] compute_pkt_idx;

    reg [31:0] pending_x1_cmd;
    reg [31:0] last_x1_result;
    reg [8:0] macro_values [0:NUM_MACROS-1];

    reg [31:0] x1_di;
    reg        x1_we;
    reg [NUM_MACROS-1:0] x1_en_mask;
    wire [NUM_MACROS-1:0] x1_ack_bus;
    wire [NUM_MACROS*32-1:0] x1_do_bus;
    wire [NUM_MACROS-1:0] scan_out_bus;

    assign ScanOutCC = |scan_out_bus;
    assign compiler_busy = (state != S_IDLE);

    reg [31:0] active_x1_do_mux;
    reg        active_x1_ack_mux;
    wire [31:0] active_x1_do = active_x1_do_mux;
    wire        active_x1_ack = active_x1_ack_mux;

    wire [31:0] status_word = {
        16'd0,
        state,
        6'd0,
        error_sticky,
        noc_flit_valid,
        result_valid,
        init_done,
        compiler_busy
    };

    genvar gi;
    generate
        for (gi = 0; gi < NUM_MACROS; gi = gi + 1) begin : gen_x1_macro
            Neuromorphic_X1_beh x1_macro (
`ifdef USE_POWER_PINS
                .VDDC1(VDDC1),
                .VDDC2(VDDC2),
                .VDDA1(VDDA1),
                .VDDA2(VDDA2),
                .VSS(VSS),
`endif
                .CLKin(wb_clk_i),
                .RSTin(wb_rst_i),
                .EN(x1_en_mask[gi]),
                .DI(x1_di),
                .W_RB(x1_we),
                .DO(x1_do_bus[(gi*32) +: 32]),
                .ack_out(x1_ack_bus[gi]),
                .ScanInCC(ScanInCC),
                .ScanInDL(ScanInDL),
                .ScanInDR(ScanInDR),
                .TM(TM),
                .ScanOutCC(scan_out_bus[gi]),
                .Iref(Iref),
                .Vcc_read(Vcc_read),
                .Vcomp(Vcomp),
                .Bias_comp2(Bias_comp2),
                .Vcc_wl_read(Vcc_wl_read),
                .Vcc_wl_set(Vcc_wl_set),
                .Vbias(Vbias),
                .Vcc_wl_reset(Vcc_wl_reset),
                .Vcc_set(Vcc_set),
                .dc_bias(dc_bias)
            );
        end
    endgenerate

    integer mux_i;
    always @* begin
        active_x1_do_mux = 32'd0;
        active_x1_ack_mux = 1'b0;
        for (mux_i = 0; mux_i < NUM_MACROS; mux_i = mux_i + 1) begin
            if (active_macro == mux_i[7:0]) begin
                active_x1_do_mux = x1_do_bus[(mux_i*32) +: 32];
                active_x1_ack_mux = x1_ack_bus[mux_i];
            end
        end
    end

    function [NUM_MACROS-1:0] onehot_macro;
        input [7:0] idx;
        integer oi;
        begin
            onehot_macro = {NUM_MACROS{1'b0}};
            for (oi = 0; oi < NUM_MACROS; oi = oi + 1) begin
                if (idx == oi)
                    onehot_macro[oi] = 1'b1;
            end
        end
    endfunction

    function [31:0] select_x1_do;
        input [7:0] idx;
        integer si;
        begin
            select_x1_do = 32'd0;
            for (si = 0; si < NUM_MACROS; si = si + 1) begin
                if (idx == si)
                    select_x1_do = x1_do_bus[(si*32) +: 32];
            end
        end
    endfunction

    function select_x1_ack;
        input [7:0] idx;
        integer ai;
        begin
            select_x1_ack = 1'b0;
            for (ai = 0; ai < NUM_MACROS; ai = ai + 1) begin
                if (idx == ai)
                    select_x1_ack = x1_ack_bus[ai];
            end
        end
    endfunction

    function [31:0] config_word;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: config_word = CFG_WORD0;
                2'd1: config_word = CFG_WORD1;
                default: config_word = CFG_WORD2;
            endcase
        end
    endfunction

    function [31:0] x1_cmd_word;
        input [1:0] mode;
        input [4:0] row;
        input [4:0] col;
        input       full_row;
        input [7:0] data;
        begin
            x1_cmd_word = {mode, row, col, 1'b0, full_row, 10'd0, data};
        end
    endfunction

    function [4:0] role_bits_per_value;
        input [3:0] role_id;
        begin
            case (role_id)
                4'd1: role_bits_per_value = 5'd8;   // gate/up: c4/f3
                4'd8: role_bits_per_value = 5'd11;  // AV: c5/f5
                default: role_bits_per_value = 5'd9; // down/q/k/v/out/qkt: c5/f3
            endcase
        end
    endfunction

    function [2:0] role_coarse_bits;
        input [3:0] role_id;
        begin
            case (role_id)
                4'd1: role_coarse_bits = 3'd4;
                default: role_coarse_bits = 3'd5;
            endcase
        end
    endfunction

    function [2:0] role_fine_bits;
        input [3:0] role_id;
        begin
            case (role_id)
                4'd8: role_fine_bits = 3'd5;
                default: role_fine_bits = 3'd3;
            endcase
        end
    endfunction

    function [6:0] tail_valid_bits;
        input [15:0] total_bits;
        reg [7:0] rem;
        begin
            rem = total_bits[7:0] & 8'h7F;
            tail_valid_bits = (rem == 8'd0) ? 7'd0 : rem[6:0];
        end
    endfunction

    function [63:0] make_header;
        input [3:0] role_id;
        input [4:0] bits_per_value;
        input [2:0] coarse_bits;
        input [2:0] fine_bits;
        input [4:0] gain_shift;
        input [9:0] value_count;
        input [11:0] tile_group_id;
        input [2:0] reduce_mode;
        input [6:0] tail_bits;
        begin
            make_header = {
                FMT_SAR_FP,
                role_id,
                bits_per_value,
                coarse_bits,
                fine_bits,
                gain_shift,
                value_count,
                tile_group_id,
                reduce_mode,
                tail_bits,
                8'h00
            };
        end
    endfunction

    function [10:0] encode_sar_value;
        input [8:0] magnitude;
        input [2:0] coarse_bits;
        input [2:0] fine_bits;
        input [4:0] gain_shift;
        integer bi;
        reg [3:0] msb_idx;
        reg [5:0] coarse_tmp;
        reg [5:0] fine_tmp;
        reg [5:0] max_coarse;
        reg [5:0] mant_shift;
        reg [10:0] payload_mask;
        reg [31:0] active_threshold;
        reg [31:0] mantissa_base;
        reg [31:0] max_mantissa;
        reg [31:0] mant_numer;
        reg [31:0] mantissa_tmp;
        reg [31:0] remainder_mask;
        reg [31:0] remainder;
        reg [31:0] half_lsb;
        reg        clipped_high;
        begin
            msb_idx = 4'd0;
            coarse_tmp = 6'd0;
            fine_tmp = 6'd0;
            clipped_high = 1'b0;

            max_coarse = (6'd1 << coarse_bits) - 6'd1;
            mantissa_base = 32'd1 << fine_bits;
            max_mantissa = (32'd1 << ({3'd0, fine_bits} + 6'd1)) - 32'd1;
            payload_mask = (11'd1 << (6'd1 + {3'd0, coarse_bits} + {3'd0, fine_bits})) - 11'd1;

            if (gain_shift == 5'd0)
                active_threshold = 32'd1;
            else if (gain_shift > 5'd9)
                active_threshold = 32'd512;
            else
                active_threshold = 32'd1 << (gain_shift - 5'd1);

            for (bi = 0; bi < 9; bi = bi + 1) begin
                if (magnitude[bi])
                    msb_idx = bi[3:0];
            end

            if ((magnitude == 9'd0) || ({23'd0, magnitude} < active_threshold)) begin
                coarse_tmp = 6'd0;
                fine_tmp = 6'd0;
            end else begin
                if ({2'd0, msb_idx} > {1'd0, gain_shift})
                    coarse_tmp = {2'd0, msb_idx} - {1'd0, gain_shift};
                else
                    coarse_tmp = 6'd0;

                if (coarse_tmp > max_coarse) begin
                    coarse_tmp = max_coarse;
                    clipped_high = 1'b1;
                end

                mant_shift = {1'd0, gain_shift} + coarse_tmp;
                mant_numer = {23'd0, magnitude} << fine_bits;
                if (mant_shift == 6'd0) begin
                    mantissa_tmp = mant_numer;
                end else begin
                    mantissa_tmp = mant_numer >> mant_shift;
                    remainder_mask = (32'd1 << mant_shift) - 32'd1;
                    remainder = mant_numer & remainder_mask;
                    half_lsb = 32'd1 << (mant_shift - 6'd1);
                    if ((remainder > half_lsb) || ((remainder == half_lsb) && mantissa_tmp[0]))
                        mantissa_tmp = mantissa_tmp + 32'd1;
                end

                if (mantissa_tmp < mantissa_base)
                    mantissa_tmp = mantissa_base;

                if (mantissa_tmp >= (32'd1 << ({3'd0, fine_bits} + 6'd1))) begin
                    if (coarse_tmp < max_coarse) begin
                        coarse_tmp = coarse_tmp + 6'd1;
                        mantissa_tmp = mantissa_base;
                    end else begin
                        clipped_high = 1'b1;
                    end
                end

                if (mantissa_tmp > max_mantissa)
                    mantissa_tmp = max_mantissa;
                if (clipped_high)
                    mantissa_tmp = max_mantissa;
                fine_tmp = mantissa_tmp - mantissa_base;
            end

            encode_sar_value = ((({6'd0, coarse_tmp[4:0]} << fine_bits) | {6'd0, fine_tmp[4:0]}) & payload_mask);
        end
    endfunction

    integer pi;
    reg [127:0] payload_next;
    reg [10:0] code_next;
    reg [4:0] bits_next;
    reg [2:0] coarse_next;
    reg [2:0] fine_next;
    reg [15:0] payload_bits_next;
    reg [15:0] total_bits_next;
    reg [63:0] header_next;

    task build_noc_packet;
        begin
            bits_next = role_bits_per_value(role_reg);
            coarse_next = role_coarse_bits(role_reg);
            fine_next = role_fine_bits(role_reg);
            payload_next = 128'd0;
            for (pi = 0; pi < NUM_MACROS; pi = pi + 1) begin
                code_next = encode_sar_value(macro_values[pi], coarse_next, fine_next, gain_shift_reg);
                payload_next = payload_next | ({{117{1'b0}}, code_next} << (pi * bits_next));
            end
            payload_bits_next = NUM_MACROS * bits_next;
            total_bits_next = 16'd64 + payload_bits_next;
            header_next = make_header(
                role_reg,
                bits_next,
                coarse_next,
                fine_next,
                gain_shift_reg,
                NUM_MACROS[9:0],
                tile_group_reg,
                REDUCE_LOCAL_ROW,
                tail_valid_bits(total_bits_next)
            );
            noc_flit_data <= {payload_next[63:0], header_next};
            noc_flit_valid <= 1'b1;
            noc_flit_last <= 1'b1;
            noc_flit_vc <= vc_reg;
            noc_dst <= dst_reg;
        end
    endtask

    wire wb_req = wbs_cyc_i & wbs_stb_i;
    wire [5:0] wb_word = wbs_adr_i[7:2];

    integer ri;
    always @(posedge wb_clk_i or posedge wb_rst_i) begin
        if (wb_rst_i) begin
            state <= S_INIT_ISSUE;
            x1_context <= CTX_INIT;
            delay_context <= CTX_INIT;
            wait_counter <= 16'd0;
            retry_counter <= 8'd0;
            row_reg <= 5'd0;
            col_reg <= 5'd0;
            data_reg <= 8'd0;
            role_reg <= 4'd2;
            gain_shift_reg <= 5'd0;
            vc_reg <= 3'd1;
            dst_reg <= 4'd0;
            tile_group_reg <= 12'd0;
            user_op <= CMD_NOP;
            user_macro <= 8'd0;
            user_full_row <= 1'b0;
            init_macro_idx <= 8'd0;
            init_pkt_idx <= 2'd0;
            active_macro <= 8'd0;
            compute_macro_idx <= 8'd0;
            compute_pkt_idx <= 2'd0;
            pending_x1_cmd <= 32'd0;
            last_x1_result <= 32'd0;
            x1_di <= 32'd0;
            x1_we <= 1'b1;
            x1_en_mask <= {NUM_MACROS{1'b0}};
            wbs_ack_o <= 1'b0;
            wbs_dat_o <= 32'd0;
            noc_flit_data <= 128'd0;
            noc_flit_valid <= 1'b0;
            noc_flit_last <= 1'b0;
            noc_flit_vc <= 3'd1;
            noc_dst <= 4'd0;
            init_done <= 1'b0;
            result_valid <= 1'b0;
            error_sticky <= 1'b0;
            for (ri = 0; ri < NUM_MACROS; ri = ri + 1) begin
                macro_values[ri] <= 9'd0;
            end
        end else begin
            wbs_ack_o <= 1'b0;
            x1_en_mask <= {NUM_MACROS{1'b0}};

            case (state)
                S_INIT_ISSUE: begin
                    active_macro <= init_macro_idx;
                    pending_x1_cmd <= config_word(init_pkt_idx);
                    x1_context <= CTX_INIT;
                    state <= S_WRITE_ISSUE;
                end

                S_WRITE_ISSUE: begin
                    x1_di <= pending_x1_cmd;
                    x1_we <= 1'b1;
                    x1_en_mask <= onehot_macro(active_macro);
                    retry_counter <= 8'd0;
                    state <= S_WAIT_WRITE_ACK;
                end

                S_WAIT_WRITE_ACK: begin
                    if (active_x1_ack) begin
                        case (x1_context)
                            CTX_INIT: begin
                                if (init_pkt_idx == 2'd2) begin
                                    init_pkt_idx <= 2'd0;
                                    if (init_macro_idx == (NUM_MACROS - 1)) begin
                                        wait_counter <= X1_INIT_DRAIN_CYCLES[15:0];
                                        delay_context <= CTX_INIT_DONE;
                                        state <= S_DELAY;
                                    end else begin
                                        init_macro_idx <= init_macro_idx + 8'd1;
                                        state <= S_INIT_ISSUE;
                                    end
                                end else begin
                                    init_pkt_idx <= init_pkt_idx + 2'd1;
                                    state <= S_INIT_ISSUE;
                                end
                            end

                            CTX_PROGRAM_DONE: begin
                                wait_counter <= X1_WRITE_DONE_CYCLES[15:0];
                                delay_context <= CTX_PROGRAM_DONE;
                                state <= S_DELAY;
                            end

                            CTX_READ_CMD: begin
                                wait_counter <= X1_READ_DONE_CYCLES[15:0];
                                delay_context <= CTX_READ_POP;
                                state <= S_DELAY;
                            end

                            CTX_COMPUTE_WRITE: begin
                                if (compute_pkt_idx == 2'd2) begin
                                    wait_counter <= X1_READ_DONE_CYCLES[15:0];
                                    delay_context <= CTX_COMPUTE_POP;
                                    state <= S_DELAY;
                                end else begin
                                    compute_pkt_idx <= compute_pkt_idx + 2'd1;
                                    pending_x1_cmd <= x1_cmd_word(X1_MODE_COMPUTE, row_reg, col_reg, user_full_row, 8'h20 + compute_pkt_idx);
                                    state <= S_WRITE_ISSUE;
                                end
                            end

                            default: begin
                                error_sticky <= 1'b1;
                                state <= S_IDLE;
                            end
                        endcase
                    end else if (retry_counter == 8'd64) begin
                        error_sticky <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        retry_counter <= retry_counter + 8'd1;
                    end
                end

                S_DELAY: begin
                    if (wait_counter != 16'd0) begin
                        wait_counter <= wait_counter - 16'd1;
                    end else begin
                        case (delay_context)
                            CTX_INIT_DONE: begin
                                init_done <= 1'b1;
                                result_valid <= 1'b0;
                                state <= S_IDLE;
                            end
                            CTX_PROGRAM_DONE: begin
                                result_valid <= 1'b1;
                                last_x1_result <= 32'd0;
                                state <= S_IDLE;
                            end
                            CTX_READ_POP: begin
                                x1_context <= CTX_READ_POP;
                                state <= S_READ_POP_ISSUE;
                            end
                            CTX_COMPUTE_POP: begin
                                x1_context <= CTX_COMPUTE_POP;
                                state <= S_READ_POP_ISSUE;
                            end
                            default: begin
                                state <= S_IDLE;
                            end
                        endcase
                    end
                end

                S_IDLE: begin
                    // Work is launched by a CSR write below.
                end

                S_DISPATCH: begin
                    if (user_op == CMD_PROGRAM) begin
                        active_macro <= user_macro;
                        pending_x1_cmd <= x1_cmd_word(X1_MODE_PROGRAM, row_reg, col_reg, 1'b0, data_reg);
                        x1_context <= CTX_PROGRAM_DONE;
                        state <= S_WRITE_ISSUE;
                    end else if (user_op == CMD_RESET) begin
                        active_macro <= user_macro;
                        pending_x1_cmd <= x1_cmd_word(X1_MODE_RESET, row_reg, col_reg, 1'b0, data_reg);
                        x1_context <= CTX_PROGRAM_DONE;
                        state <= S_WRITE_ISSUE;
                    end else if (user_op == CMD_READ) begin
                        active_macro <= user_macro;
                        pending_x1_cmd <= x1_cmd_word(X1_MODE_READ, row_reg, col_reg, 1'b0, 8'd0);
                        x1_context <= CTX_READ_CMD;
                        state <= S_WRITE_ISSUE;
                    end else if (user_op == CMD_COMPUTE) begin
                        compute_macro_idx <= 8'd0;
                        state <= S_COMPUTE_PREP;
                    end else begin
                        error_sticky <= 1'b1;
                        result_valid <= 1'b1;
                        state <= S_IDLE;
                    end
                end

                S_COMPUTE_PREP: begin
                    active_macro <= compute_macro_idx;
                    compute_pkt_idx <= 2'd0;
                    pending_x1_cmd <= x1_cmd_word(X1_MODE_COMPUTE, row_reg, col_reg, user_full_row, 8'h10);
                    x1_context <= CTX_COMPUTE_WRITE;
                    state <= S_WRITE_ISSUE;
                end

                S_READ_POP_ISSUE: begin
                    x1_we <= 1'b0;
                    x1_en_mask <= onehot_macro(active_macro);
                    retry_counter <= 8'd0;
                    state <= S_WAIT_READ_ACK;
                end

                S_WAIT_READ_ACK: begin
                    if (active_x1_ack) begin
                        state <= S_CAPTURE_READ;
                    end else if (retry_counter == 8'd32) begin
                        state <= S_READ_POP_ISSUE;
                    end else begin
                        retry_counter <= retry_counter + 8'd1;
                    end
                end

                S_CAPTURE_READ: begin
                    last_x1_result <= active_x1_do;
                    if (x1_context == CTX_READ_POP) begin
                        result_valid <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        macro_values[active_macro] <= active_x1_do[8:0];
                        if (active_macro == (NUM_MACROS - 1)) begin
                            build_noc_packet();
                            result_valid <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            compute_macro_idx <= active_macro + 8'd1;
                            state <= S_COMPUTE_PREP;
                        end
                    end
                end

                S_DONE: begin
                    result_valid <= 1'b1;
                    state <= S_IDLE;
                end

                default: begin
                    error_sticky <= 1'b1;
                    state <= S_IDLE;
                end
            endcase

            if (wb_req && !wbs_ack_o) begin
                wbs_ack_o <= 1'b1;
                if (wbs_we_i) begin
                    case (wb_word)
                        REG_ADDR: begin
                            row_reg <= wbs_dat_i[4:0];
                            col_reg <= wbs_dat_i[12:8];
                        end
                        REG_DATA: begin
                            data_reg <= wbs_dat_i[7:0];
                        end
                        REG_ROLE: begin
                            role_reg <= wbs_dat_i[3:0];
                            gain_shift_reg <= wbs_dat_i[8:4];
                            vc_reg <= wbs_dat_i[11:9];
                            dst_reg <= wbs_dat_i[15:12];
                            tile_group_reg <= wbs_dat_i[27:16];
                        end
                        REG_COMMAND: begin
                            if (wbs_dat_i[31]) begin
                                if ((state == S_IDLE) && init_done) begin
                                    user_op <= wbs_dat_i[3:0];
                                    user_macro <= (wbs_dat_i[7:4] < NUM_MACROS) ? {4'd0, wbs_dat_i[7:4]} : 8'd0;
                                    user_full_row <= wbs_dat_i[9];
                                    result_valid <= 1'b0;
                                    noc_flit_valid <= 1'b0;
                                    noc_flit_last <= 1'b0;
                                    if (wbs_dat_i[7:4] >= NUM_MACROS)
                                        error_sticky <= 1'b1;
                                    state <= S_DISPATCH;
                                end else begin
                                    error_sticky <= 1'b1;
                                end
                            end
                        end
                        REG_CLEAR: begin
                            if (wbs_dat_i[0])
                                result_valid <= 1'b0;
                            if (wbs_dat_i[1])
                                noc_flit_valid <= 1'b0;
                            if (wbs_dat_i[2])
                                error_sticky <= 1'b0;
                        end
                        default: begin
                        end
                    endcase
                end else begin
                    case (wb_word)
                        REG_VERSION: wbs_dat_o <= 32'h5831_4E43; // X1NC
                        REG_CONFIG:  wbs_dat_o <= ((NUM_MACROS & 32'hFF) << 16) | 32'h0000_2020;
                        REG_STATUS:  wbs_dat_o <= status_word;
                        REG_COMMAND: wbs_dat_o <= {22'd0, user_full_row, 1'b0, user_macro[3:0], user_op};
                        REG_ADDR:    wbs_dat_o <= {19'd0, col_reg, 3'd0, row_reg};
                        REG_DATA:    wbs_dat_o <= {24'd0, data_reg};
                        REG_ROLE:    wbs_dat_o <= {4'd0, tile_group_reg, dst_reg, vc_reg, gain_shift_reg, role_reg};
                        REG_RESULT:  wbs_dat_o <= last_x1_result;
                        REG_FLIT0:   wbs_dat_o <= noc_flit_data[31:0];
                        REG_FLIT1:   wbs_dat_o <= noc_flit_data[63:32];
                        REG_FLIT2:   wbs_dat_o <= noc_flit_data[95:64];
                        REG_FLIT3:   wbs_dat_o <= noc_flit_data[127:96];
                        default:     wbs_dat_o <= 32'd0;
                    endcase
                end
            end
        end
    end

endmodule

`default_nettype wire
