`default_nettype none

// X1-only memory compiler and SAR-TDC NoC endpoint.
// This module intentionally instantiates only the submit behavioral X1 macro.
module x1_memory_compiler_noc #(
    parameter integer NUM_MACROS = 4,
    parameter integer NUM_NOC_NODES = 4,
    parameter integer X1_WRITE_DONE_CYCLES = 512,
    parameter integer X1_READ_DONE_CYCLES = 256,
    parameter integer X1_INIT_DRAIN_CYCLES = 512,
    parameter integer X1_ANALOG_WEIGHT_MODE = 0
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

    output [127:0] noc_flit_data,
    output         noc_flit_valid,
    output         noc_flit_last,
    output [2:0]   noc_flit_vc,
    output [3:0]   noc_dst,
    output [NUM_NOC_NODES-1:0] noc_node_valid,
    output [NUM_NOC_NODES*128-1:0] noc_node_flit_data,
    output [NUM_NOC_NODES*3-1:0]   noc_node_vc,
    output [NUM_NOC_NODES*4-1:0]   noc_node_src,
    output             compiler_busy,
    output reg         init_done,
    output reg         result_valid,
    output reg         error_sticky,

    output reg         periph_rd_req_valid,
    output reg [31:0]  periph_rd_req_addr,
    input              periph_rd_req_ready,
    input              periph_rd_rsp_valid,
    input      [31:0]  periph_rd_rsp_data,
    output reg         periph_rd_rsp_ready,

    output reg         periph_wr_req_valid,
    output reg [31:0]  periph_wr_req_addr,
    output reg [31:0]  periph_wr_req_data,
    output reg [3:0]   periph_wr_req_strb,
    input              periph_wr_req_ready
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
    localparam [5:0] REG_DESC_INDEX  = 6'd13;
    localparam [5:0] REG_DESC_DATA   = 6'd14;
    localparam [5:0] REG_DESC_CTRL   = 6'd15;
    localparam [5:0] REG_DESC_STATUS = 6'd16;
    localparam [5:0] REG_DESC_RESULT = 6'd17;
    localparam [5:0] REG_DESC_FETCH_ADDR = 6'd18;
    localparam [5:0] REG_WINDOW_INDEX = 6'd19;
    localparam [5:0] REG_WINDOW_DATA  = 6'd20;

    localparam [3:0] CMD_NOP     = 4'd0;
    localparam [3:0] CMD_PROGRAM = 4'd1;
    localparam [3:0] CMD_RESET   = 4'd2;
    localparam [3:0] CMD_READ    = 4'd3;
    localparam [3:0] CMD_COMPUTE = 4'd4;

    localparam [7:0] DESC_MAGIC = 8'hA1;
    localparam [3:0] DESC_VERSION = 4'd1;
    localparam [7:0] DESC_OP_DMA_LOAD = 8'd1;
    localparam [7:0] DESC_OP_DMA_STORE = 8'd2;
    localparam [7:0] DESC_OP_PROGRAM_X1_WINDOW = 8'd3;
    localparam [7:0] DESC_OP_EXECUTE_TILE = 8'd4;
    localparam [7:0] DESC_OP_NOC_SEND = 8'd5;
    localparam [7:0] DESC_OP_REDUCE = 8'd6;
    localparam [7:0] DESC_OP_EVICT_WINDOW = 8'd7;
    localparam [7:0] DESC_OP_FENCE = 8'd8;
    localparam [7:0] DESC_OP_PEER_LOAD = 8'd9;

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
    localparam [4:0] S_DESC_FETCH_REQ   = 5'd11;
    localparam [4:0] S_DESC_FETCH_WAIT  = 5'd12;
    localparam [4:0] S_DESC_VALIDATE    = 5'd13;
    localparam [4:0] S_DESC_PROGRAM_PREP = 5'd14;
    localparam [4:0] S_DMA_LOAD_REQ     = 5'd15;
    localparam [4:0] S_DMA_LOAD_WAIT    = 5'd16;
    localparam [4:0] S_DMA_STORE_REQ    = 5'd17;

    localparam [3:0] CTX_INIT             = 4'd0;
    localparam [3:0] CTX_PROGRAM_DONE     = 4'd1;
    localparam [3:0] CTX_READ_CMD         = 4'd2;
    localparam [3:0] CTX_READ_POP         = 4'd3;
    localparam [3:0] CTX_COMPUTE_WRITE    = 4'd4;
    localparam [3:0] CTX_COMPUTE_POP      = 4'd5;
    localparam [3:0] CTX_INIT_DONE        = 4'd6;
    localparam [3:0] CTX_DESC_PROGRAM     = 4'd7;
    localparam [3:0] CTX_DMA_LOAD_PROGRAM = 4'd8;

    localparam [31:0] CFG_WORD0 = 32'hA203_C40F;
    localparam [31:0] CFG_WORD1 = 32'h0F03_0D43;
    localparam [31:0] CFG_WORD2 = {2'b01, 3'b000, 7'd32, 10'd3, 10'd3};

    reg [4:0] state;
    reg [3:0] x1_context;
    reg [3:0] delay_context;
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

    reg [127:0] endpoint_noc_flit_data;
    reg         endpoint_noc_flit_valid;
    reg         endpoint_noc_flit_last;
    reg [2:0]   endpoint_noc_flit_vc;
    reg [3:0]   endpoint_noc_dst;

    x1_noc_router_fabric #(
        .NUM_NODES(NUM_NOC_NODES)
    ) noc_fabric (
        .clk(wb_clk_i),
        .rst(wb_rst_i),
        .in_flit(endpoint_noc_flit_data),
        .in_valid(endpoint_noc_flit_valid),
        .in_last(endpoint_noc_flit_last),
        .in_vc(endpoint_noc_flit_vc),
        .in_src(4'd0),
        .in_dst(endpoint_noc_dst),
        .route_flit(noc_flit_data),
        .route_valid(noc_flit_valid),
        .route_last(noc_flit_last),
        .route_vc(noc_flit_vc),
        .route_dst(noc_dst),
        .node_valid(noc_node_valid),
        .node_flit_data(noc_node_flit_data),
        .node_vc(noc_node_vc),
        .node_src(noc_node_src)
    );

    reg [3:0] desc_index;
    reg [31:0] desc_words [0:15];
    reg desc_busy;
    reg desc_done;
    reg desc_error;
    reg desc_active;
    reg [7:0] desc_last_op;
    reg [15:0] desc_exec_count;
    reg [31:0] desc_result_word;
    reg [31:0] desc_checksum_calc;
    reg [31:0] desc_fetch_addr;
    reg [3:0] desc_fetch_index;
    reg [3:0] window_index;
    reg [31:0] window_words [0:15];
    reg [31:0] result_sram_words [0:15];
    reg [7:0] window_program_count;
    reg [7:0] window_program_idx;
    reg [7:0] dma_word_count;
    reg [7:0] dma_word_idx;

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
        8'd0,
        desc_error,
        desc_done,
        desc_busy,
        5'd0,
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
            Neuromorphic_X1_beh #(
                .ANALOG_WEIGHT_MODE(X1_ANALOG_WEIGHT_MODE)
            ) x1_macro (
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

    integer checksum_i;
    always @* begin
        desc_checksum_calc = 32'h5831_5550;
        for (checksum_i = 0; checksum_i < 14; checksum_i = checksum_i + 1) begin
            desc_checksum_calc = {desc_checksum_calc[26:0], desc_checksum_calc[31:27]};
            desc_checksum_calc = desc_checksum_calc ^ desc_words[checksum_i];
        end
    end

    wire desc_magic_ok = (desc_words[0][31:24] === DESC_MAGIC);
    wire desc_version_ok = (desc_words[0][23:20] === DESC_VERSION);
    wire desc_checksum_ok = (desc_checksum_calc === desc_words[14]);
    wire [7:0] desc_op = desc_words[0][19:12];
    wire [31:0] desc_fetch_word_addr = desc_fetch_addr + {26'd0, desc_fetch_index, 2'b00};
    wire [31:0] desc_program_entries = {2'd0, desc_words[7][31:2]};
    wire [7:0] desc_transfer_words = (desc_words[7][1:0] == 2'd0) ? desc_words[7][9:2] : (desc_words[7][9:2] + 8'd1);
    wire desc_transfer_count_ok = (desc_transfer_words != 8'd0) && (desc_transfer_words <= 8'd16);
    wire [31:0] dma_load_word_addr = desc_words[2] + {22'd0, dma_word_idx, 2'b00};
    wire [31:0] dma_store_word_addr = desc_words[5] + {22'd0, dma_word_idx, 2'b00};
    wire [3:0] dma_load_dst_index = desc_words[5][5:2] + dma_word_idx[3:0];
    wire [3:0] dma_store_src_index = desc_words[2][5:2] + dma_word_idx[3:0];

    reg [31:0] dma_store_data_mux;
    always @* begin
        if (desc_words[2][15:12] == 4'h2)
            dma_store_data_mux = result_sram_words[dma_store_src_index];
        else
            dma_store_data_mux = window_words[dma_store_src_index];
    end

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
        reg [8:0] fine_mask;
        begin
            msb_idx = 4'd0;
            for (bi = 0; bi < 9; bi = bi + 1) begin
                if (magnitude[bi])
                    msb_idx = bi[3:0];
            end

            if (magnitude == 9'd0) begin
                coarse_tmp = 6'd0;
                fine_tmp = 6'd0;
            end else begin
                coarse_tmp = {2'd0, msb_idx} + {1'd0, gain_shift};
                if (coarse_tmp > ((6'd1 << coarse_bits) - 6'd1))
                    coarse_tmp = (6'd1 << coarse_bits) - 6'd1;

                fine_mask = (9'd1 << fine_bits) - 9'd1;
                if (msb_idx >= (fine_bits - 1'b1))
                    fine_tmp = (magnitude >> (msb_idx - (fine_bits - 1'b1))) & fine_mask;
                else
                    fine_tmp = (magnitude << ((fine_bits - 1'b1) - msb_idx)) & fine_mask;
            end

            encode_sar_value = ({1'b0, coarse_tmp[4:0], fine_tmp[4:0]}) & ((11'd1 << (1 + coarse_bits + fine_bits)) - 11'd1);
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
            endpoint_noc_flit_data <= {payload_next[63:0], header_next};
            endpoint_noc_flit_valid <= 1'b1;
            endpoint_noc_flit_last <= 1'b1;
            endpoint_noc_flit_vc <= vc_reg;
            endpoint_noc_dst <= dst_reg;
        end
    endtask

    task launch_descriptor_from_words;
        input fetched_start;
        begin
            desc_done <= 1'b0;
            desc_error <= 1'b0;
            desc_result_word <= 32'd0;
            desc_last_op <= desc_op;
            if (fetched_start || ((state == S_IDLE) && init_done && !desc_busy)) begin
                if (!desc_magic_ok || !desc_version_ok || !desc_checksum_ok) begin
                    desc_busy <= 1'b0;
                    desc_active <= 1'b0;
                    desc_error <= 1'b1;
                    desc_done <= 1'b1;
                    desc_result_word <= 32'hBAD0_0001;
                    error_sticky <= 1'b1;
                    state <= S_IDLE;
                end else if (desc_op == DESC_OP_EXECUTE_TILE) begin
                    row_reg <= desc_words[10][20:16];
                    col_reg <= desc_words[10][4:0];
                    role_reg <= desc_words[11][27:24];
                    vc_reg <= desc_words[11][18:16];
                    dst_reg <= desc_words[11][11:8];
                    tile_group_reg <= desc_words[9][27:16];
                    user_full_row <= desc_words[0][0];
                    result_valid <= 1'b0;
                    endpoint_noc_flit_valid <= 1'b0;
                    endpoint_noc_flit_last <= 1'b0;
                    desc_busy <= 1'b1;
                    desc_active <= 1'b1;
                    compute_macro_idx <= 8'd0;
                    state <= S_COMPUTE_PREP;
                end else if (desc_op == DESC_OP_PROGRAM_X1_WINDOW) begin
                    if ((desc_program_entries == 32'd0) || (desc_program_entries > 32'd16)) begin
                        desc_busy <= 1'b0;
                        desc_active <= 1'b0;
                        desc_error <= 1'b1;
                        desc_done <= 1'b1;
                        desc_result_word <= 32'hBAD0_0004;
                        error_sticky <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        result_valid <= 1'b0;
                        desc_busy <= 1'b1;
                        desc_active <= 1'b1;
                        window_program_count <= {3'd0, desc_words[7][6:2]};
                        window_program_idx <= 8'd0;
                        state <= S_DESC_PROGRAM_PREP;
                    end
                end else if (desc_op == DESC_OP_DMA_LOAD) begin
                    if (!desc_transfer_count_ok) begin
                        desc_busy <= 1'b0;
                        desc_active <= 1'b0;
                        desc_error <= 1'b1;
                        desc_done <= 1'b1;
                        desc_result_word <= 32'hBAD0_0006;
                        error_sticky <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        desc_busy <= 1'b1;
                        desc_active <= 1'b1;
                        dma_word_count <= desc_transfer_words;
                        dma_word_idx <= 8'd0;
                        state <= S_DMA_LOAD_REQ;
                    end
                end else if (desc_op == DESC_OP_NOC_SEND) begin
                    if ((desc_transfer_words == 8'd0) || (desc_transfer_words > 8'd4)) begin
                        desc_busy <= 1'b0;
                        desc_active <= 1'b0;
                        desc_error <= 1'b1;
                        desc_done <= 1'b1;
                        desc_result_word <= 32'hBAD0_0007;
                        error_sticky <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        if (desc_transfer_words > 8'd0)
                            result_sram_words[desc_words[5][5:2]] <= noc_flit_data[31:0];
                        if (desc_transfer_words > 8'd1)
                            result_sram_words[desc_words[5][5:2] + 4'd1] <= noc_flit_data[63:32];
                        if (desc_transfer_words > 8'd2)
                            result_sram_words[desc_words[5][5:2] + 4'd2] <= noc_flit_data[95:64];
                        if (desc_transfer_words > 8'd3)
                            result_sram_words[desc_words[5][5:2] + 4'd3] <= noc_flit_data[127:96];
                        desc_busy <= 1'b0;
                        desc_active <= 1'b0;
                        desc_done <= 1'b1;
                        desc_exec_count <= desc_exec_count + 16'd1;
                        desc_result_word <= {16'd0, desc_transfer_words};
                        state <= S_IDLE;
                    end
                end else if (desc_op == DESC_OP_DMA_STORE) begin
                    if (!desc_transfer_count_ok) begin
                        desc_busy <= 1'b0;
                        desc_active <= 1'b0;
                        desc_error <= 1'b1;
                        desc_done <= 1'b1;
                        desc_result_word <= 32'hBAD0_0008;
                        error_sticky <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        desc_busy <= 1'b1;
                        desc_active <= 1'b1;
                        dma_word_count <= desc_transfer_words;
                        dma_word_idx <= 8'd0;
                        state <= S_DMA_STORE_REQ;
                    end
                end else if (
                    (desc_op == DESC_OP_REDUCE) ||
                    (desc_op == DESC_OP_EVICT_WINDOW) ||
                    (desc_op == DESC_OP_FENCE) ||
                    (desc_op == DESC_OP_PEER_LOAD)
                ) begin
                    desc_busy <= 1'b0;
                    desc_active <= 1'b0;
                    desc_done <= 1'b1;
                    desc_exec_count <= desc_exec_count + 16'd1;
                    desc_result_word <= {24'd0, desc_op};
                    state <= S_IDLE;
                end else begin
                    desc_busy <= 1'b0;
                    desc_active <= 1'b0;
                    desc_error <= 1'b1;
                    desc_done <= 1'b1;
                    desc_result_word <= 32'hBAD0_0002;
                    error_sticky <= 1'b1;
                    state <= S_IDLE;
                end
            end else begin
                desc_busy <= 1'b0;
                desc_active <= 1'b0;
                desc_error <= 1'b1;
                desc_done <= 1'b1;
                desc_result_word <= 32'hBAD0_0003;
                error_sticky <= 1'b1;
                state <= S_IDLE;
            end
        end
    endtask

    wire wb_req = wbs_cyc_i & wbs_stb_i;
    wire [5:0] wb_word = wbs_adr_i[7:2];

    integer ri;
    integer di;
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
            endpoint_noc_flit_data <= 128'd0;
            endpoint_noc_flit_valid <= 1'b0;
            endpoint_noc_flit_last <= 1'b0;
            endpoint_noc_flit_vc <= 3'd1;
            endpoint_noc_dst <= 4'd0;
            init_done <= 1'b0;
            result_valid <= 1'b0;
            error_sticky <= 1'b0;
            desc_index <= 4'd0;
            desc_busy <= 1'b0;
            desc_done <= 1'b0;
            desc_error <= 1'b0;
            desc_active <= 1'b0;
            desc_last_op <= 8'd0;
            desc_exec_count <= 16'd0;
            desc_result_word <= 32'd0;
            desc_fetch_addr <= 32'd0;
            desc_fetch_index <= 4'd0;
            window_index <= 4'd0;
            window_program_count <= 8'd0;
            window_program_idx <= 8'd0;
            dma_word_count <= 8'd0;
            dma_word_idx <= 8'd0;
            periph_rd_req_valid <= 1'b0;
            periph_rd_req_addr <= 32'd0;
            periph_rd_rsp_ready <= 1'b0;
            periph_wr_req_valid <= 1'b0;
            periph_wr_req_addr <= 32'd0;
            periph_wr_req_data <= 32'd0;
            periph_wr_req_strb <= 4'd0;
            for (ri = 0; ri < NUM_MACROS; ri = ri + 1) begin
                macro_values[ri] <= 9'd0;
            end
            for (di = 0; di < 16; di = di + 1) begin
                desc_words[di] <= 32'd0;
                window_words[di] <= 32'd0;
                result_sram_words[di] <= 32'd0;
            end
        end else begin
            wbs_ack_o <= 1'b0;
            x1_en_mask <= {NUM_MACROS{1'b0}};
            periph_rd_req_valid <= 1'b0;
            periph_rd_rsp_ready <= 1'b0;
            periph_wr_req_valid <= 1'b0;

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

                            CTX_DESC_PROGRAM: begin
                                wait_counter <= X1_WRITE_DONE_CYCLES[15:0];
                                delay_context <= CTX_DESC_PROGRAM;
                                state <= S_DELAY;
                            end

                            CTX_DMA_LOAD_PROGRAM: begin
                                wait_counter <= X1_WRITE_DONE_CYCLES[15:0];
                                delay_context <= CTX_DMA_LOAD_PROGRAM;
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
                            CTX_DESC_PROGRAM: begin
                                if ((window_program_idx + 8'd1) >= window_program_count) begin
                                    result_valid <= 1'b1;
                                    last_x1_result <= 32'd0;
                                    desc_busy <= 1'b0;
                                    desc_done <= 1'b1;
                                    desc_active <= 1'b0;
                                    desc_exec_count <= desc_exec_count + 16'd1;
                                    desc_result_word <= {16'd0, window_program_count};
                                    state <= S_IDLE;
                                end else begin
                                    window_program_idx <= window_program_idx + 8'd1;
                                    state <= S_DESC_PROGRAM_PREP;
                                end
                            end
                            CTX_DMA_LOAD_PROGRAM: begin
                                if ((dma_word_idx + 8'd1) >= dma_word_count) begin
                                    result_valid <= 1'b1;
                                    last_x1_result <= 32'd0;
                                    desc_busy <= 1'b0;
                                    desc_done <= 1'b1;
                                    desc_active <= 1'b0;
                                    desc_exec_count <= desc_exec_count + 16'd1;
                                    desc_result_word <= {16'd0, dma_word_count};
                                    state <= S_IDLE;
                                end else begin
                                    dma_word_idx <= dma_word_idx + 8'd1;
                                    state <= S_DMA_LOAD_REQ;
                                end
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

                S_DESC_FETCH_REQ: begin
                    periph_rd_req_valid <= 1'b1;
                    periph_rd_req_addr <= desc_fetch_word_addr;
                    if (periph_rd_req_valid && periph_rd_req_ready) begin
                        periph_rd_req_valid <= 1'b0;
                        periph_rd_rsp_ready <= 1'b1;
                        state <= S_DESC_FETCH_WAIT;
                    end
                end

                S_DESC_FETCH_WAIT: begin
                    periph_rd_rsp_ready <= 1'b1;
                    if (periph_rd_rsp_valid) begin
                        desc_words[desc_fetch_index] <= periph_rd_rsp_data;
                        if (desc_fetch_index == 4'd15) begin
                            desc_index <= 4'd0;
                            state <= S_DESC_VALIDATE;
                        end else begin
                            desc_fetch_index <= desc_fetch_index + 4'd1;
                            state <= S_DESC_FETCH_REQ;
                        end
                    end
                end

                S_DESC_VALIDATE: begin
                    launch_descriptor_from_words(1'b1);
                end

                S_DMA_LOAD_REQ: begin
                    periph_rd_req_valid <= 1'b1;
                    periph_rd_req_addr <= dma_load_word_addr;
                    if (periph_rd_req_valid && periph_rd_req_ready) begin
                        periph_rd_req_valid <= 1'b0;
                        periph_rd_rsp_ready <= 1'b1;
                        state <= S_DMA_LOAD_WAIT;
                    end
                end

                S_DMA_LOAD_WAIT: begin
                    periph_rd_rsp_ready <= 1'b1;
                    if (periph_rd_rsp_valid) begin
                        if (periph_rd_rsp_data[23:16] >= NUM_MACROS) begin
                            desc_busy <= 1'b0;
                            desc_active <= 1'b0;
                            desc_done <= 1'b1;
                            desc_error <= 1'b1;
                            desc_result_word <= 32'hBAD0_0009;
                            error_sticky <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            active_macro <= periph_rd_rsp_data[23:16];
                            row_reg <= periph_rd_rsp_data[4:0];
                            col_reg <= periph_rd_rsp_data[12:8];
                            data_reg <= periph_rd_rsp_data[31:24];
                            pending_x1_cmd <= x1_cmd_word(
                                X1_MODE_PROGRAM,
                                periph_rd_rsp_data[4:0],
                                periph_rd_rsp_data[12:8],
                                1'b0,
                                periph_rd_rsp_data[31:24]
                            );
                            x1_context <= CTX_DMA_LOAD_PROGRAM;
                            state <= S_WRITE_ISSUE;
                        end
                    end
                end

                S_DMA_STORE_REQ: begin
                    periph_wr_req_valid <= 1'b1;
                    periph_wr_req_addr <= dma_store_word_addr;
                    periph_wr_req_data <= dma_store_data_mux;
                    periph_wr_req_strb <= 4'hF;
                    if (periph_wr_req_valid && periph_wr_req_ready) begin
                        periph_wr_req_valid <= 1'b0;
                        if ((dma_word_idx + 8'd1) >= dma_word_count) begin
                            desc_busy <= 1'b0;
                            desc_active <= 1'b0;
                            desc_done <= 1'b1;
                            desc_exec_count <= desc_exec_count + 16'd1;
                            desc_result_word <= {16'd0, dma_word_count};
                            state <= S_IDLE;
                        end else begin
                            dma_word_idx <= dma_word_idx + 8'd1;
                            state <= S_DMA_STORE_REQ;
                        end
                    end
                end

                S_DESC_PROGRAM_PREP: begin
                    if (window_words[window_program_idx[3:0]][23:16] >= NUM_MACROS) begin
                        desc_busy <= 1'b0;
                        desc_active <= 1'b0;
                        desc_error <= 1'b1;
                        desc_done <= 1'b1;
                        desc_result_word <= 32'hBAD0_0005;
                        error_sticky <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        active_macro <= window_words[window_program_idx[3:0]][23:16];
                        row_reg <= window_words[window_program_idx[3:0]][4:0];
                        col_reg <= window_words[window_program_idx[3:0]][12:8];
                        data_reg <= window_words[window_program_idx[3:0]][31:24];
                        pending_x1_cmd <= x1_cmd_word(
                            X1_MODE_PROGRAM,
                            window_words[window_program_idx[3:0]][4:0],
                            window_words[window_program_idx[3:0]][12:8],
                            1'b0,
                            window_words[window_program_idx[3:0]][31:24]
                        );
                        x1_context <= CTX_DESC_PROGRAM;
                        state <= S_WRITE_ISSUE;
                    end
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
                            if (desc_active) begin
                                desc_busy <= 1'b0;
                                desc_done <= 1'b1;
                                desc_active <= 1'b0;
                                desc_exec_count <= desc_exec_count + 16'd1;
                                desc_result_word <= {16'd0, tile_group_reg};
                            end
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
                                    endpoint_noc_flit_valid <= 1'b0;
                                    endpoint_noc_flit_last <= 1'b0;
                                    if (wbs_dat_i[7:4] >= NUM_MACROS)
                                        error_sticky <= 1'b1;
                                    state <= S_DISPATCH;
                                end else begin
                                    error_sticky <= 1'b1;
                                end
                            end
                        end
                        REG_DESC_INDEX: begin
                            desc_index <= wbs_dat_i[3:0];
                        end
                        REG_DESC_DATA: begin
                            desc_words[desc_index] <= wbs_dat_i;
                            desc_index <= desc_index + 4'd1;
                        end
                        REG_DESC_CTRL: begin
                            if (wbs_dat_i[1]) begin
                                desc_done <= 1'b0;
                                desc_error <= 1'b0;
                                desc_busy <= 1'b0;
                                desc_active <= 1'b0;
                                desc_result_word <= 32'd0;
                            end
                            if (wbs_dat_i[0]) begin
                                launch_descriptor_from_words(1'b0);
                            end
                            if (wbs_dat_i[2]) begin
                                desc_done <= 1'b0;
                                desc_error <= 1'b0;
                                desc_result_word <= 32'd0;
                                if ((state == S_IDLE) && init_done && !desc_busy) begin
                                    desc_busy <= 1'b1;
                                    desc_active <= 1'b0;
                                    desc_fetch_index <= 4'd0;
                                    desc_index <= 4'd0;
                                    state <= S_DESC_FETCH_REQ;
                                end else begin
                                    desc_error <= 1'b1;
                                    desc_done <= 1'b1;
                                    desc_result_word <= 32'hBAD0_0003;
                                    error_sticky <= 1'b1;
                                end
                            end
                        end
                        REG_DESC_FETCH_ADDR: begin
                            desc_fetch_addr <= wbs_dat_i;
                        end
                        REG_WINDOW_INDEX: begin
                            window_index <= wbs_dat_i[3:0];
                        end
                        REG_WINDOW_DATA: begin
                            window_words[window_index] <= wbs_dat_i;
                            window_index <= window_index + 4'd1;
                        end
                        REG_CLEAR: begin
                            if (wbs_dat_i[0])
                                result_valid <= 1'b0;
                            if (wbs_dat_i[1])
                                endpoint_noc_flit_valid <= 1'b0;
                            if (wbs_dat_i[2])
                                error_sticky <= 1'b0;
                            if (wbs_dat_i[3]) begin
                                desc_done <= 1'b0;
                                desc_error <= 1'b0;
                                desc_busy <= 1'b0;
                                desc_active <= 1'b0;
                            end
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
                        REG_DESC_INDEX:  wbs_dat_o <= {28'd0, desc_index};
                        REG_DESC_DATA:   wbs_dat_o <= desc_words[desc_index];
                        REG_DESC_CTRL:   wbs_dat_o <= {31'd0, desc_busy};
                        REG_DESC_STATUS: wbs_dat_o <= {desc_exec_count, desc_last_op, 4'd0, desc_checksum_ok, desc_error, desc_done, desc_busy};
                        REG_DESC_RESULT: wbs_dat_o <= desc_result_word;
                        REG_DESC_FETCH_ADDR: wbs_dat_o <= desc_fetch_addr;
                        REG_WINDOW_INDEX: wbs_dat_o <= {28'd0, window_index};
                        REG_WINDOW_DATA:  wbs_dat_o <= window_words[window_index];
                        default:     wbs_dat_o <= 32'd0;
                    endcase
                end
            end
        end
    end

endmodule

module x1_noc_router_fabric #(
    parameter integer NUM_NODES = 4
) (
    input         clk,
    input         rst,
    input  [127:0] in_flit,
    input         in_valid,
    input         in_last,
    input  [2:0]  in_vc,
    input  [3:0]  in_src,
    input  [3:0]  in_dst,
    output [127:0] route_flit,
    output        route_valid,
    output        route_last,
    output [2:0]  route_vc,
    output [3:0]  route_dst,
    output reg [NUM_NODES-1:0]       node_valid,
    output reg [NUM_NODES*128-1:0]   node_flit_data,
    output reg [NUM_NODES*3-1:0]     node_vc,
    output reg [NUM_NODES*4-1:0]     node_src
);
    function [3:0] route_index;
        input [3:0] dst;
        begin
            if (dst < NUM_NODES[3:0])
                route_index = dst;
            else
                route_index = dst % NUM_NODES[3:0];
        end
    endfunction

    wire [3:0] dst_index = route_index(in_dst);
    integer clear_i;

    assign route_flit = in_flit;
    assign route_valid = in_valid;
    assign route_last = in_last;
    assign route_vc = in_vc;
    assign route_dst = dst_index;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            node_valid <= {NUM_NODES{1'b0}};
            node_flit_data <= {(NUM_NODES*128){1'b0}};
            node_vc <= {(NUM_NODES*3){1'b0}};
            node_src <= {(NUM_NODES*4){1'b0}};
        end else if (in_valid) begin
            node_valid[dst_index] <= 1'b1;
            node_flit_data[(dst_index*128) +: 128] <= in_flit;
            node_vc[(dst_index*3) +: 3] <= in_vc;
            node_src[(dst_index*4) +: 4] <= in_src;
        end
    end
endmodule

`default_nettype wire
