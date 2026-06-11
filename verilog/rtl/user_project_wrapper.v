`default_nettype none
module user_project_wrapper #(
    parameter BITS = 32
) (
`ifdef USE_POWER_PINS
    inout vdda1, inout vdda2,
    inout vssa1, inout vssa2,
    inout vccd1, inout vccd2,
    inout vssd1, inout vssd2,
`endif

    input         wb_clk_i,
    input         wb_rst_i,
    input         wbs_stb_i,
    input         wbs_cyc_i,
    input         wbs_we_i,
    input  [3:0]  wbs_sel_i,
    input  [31:0] wbs_dat_i,
    input  [31:0] wbs_adr_i,
    output        wbs_ack_o,
    output [31:0] wbs_dat_o,

    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    inout  [`MPRJ_IO_PADS-10:0] analog_io,
    input   user_clock2,
    output [2:0] user_irq
);

    // External GPIO byte stream pinout:
    //   host -> chip: io_in[0] valid, io_in[8:1] data, io_in[9] ready for chip TX
    //   chip -> host: io_out[28] valid, io_out[36:29] data, io_out[37] ready for host RX
    wire gpio_rx_valid = io_in[0];
    wire [7:0] gpio_rx_data = io_in[8:1];
    wire gpio_tx_ready = io_in[9];
    wire gpio_tx_valid;
    wire [7:0] gpio_tx_data;
    wire gpio_rx_ready;

    wire scan_in_cc = io_in[14];
    wire scan_in_dl = io_in[15];
    wire scan_in_dr = io_in[16];
    wire tm         = io_in[17];
    wire scan_out_cc;

    wire [127:0] noc_flit_data;
    wire         noc_flit_valid;
    wire         noc_flit_last;
    wire [2:0]   noc_flit_vc;
    wire [3:0]   noc_dst;
    wire [3:0]   noc_node_valid;
    wire [511:0] noc_node_flit_data;
    wire [11:0]  noc_node_vc;
    wire [15:0]  noc_node_src;
    wire         compiler_busy;
    wire         init_done;
    wire         result_valid;
    wire         error_sticky;
    wire         periph_rd_req_valid;
    wire [31:0]  periph_rd_req_addr;
    wire         periph_rd_req_ready;
    wire         periph_rd_rsp_valid;
    wire [31:0]  periph_rd_rsp_data;
    wire         periph_rd_rsp_ready;
    wire         periph_wr_req_valid;
    wire [31:0]  periph_wr_req_addr;
    wire [31:0]  periph_wr_req_data;
    wire [3:0]   periph_wr_req_strb;
    wire         periph_wr_req_ready;

    gpio_dma_stream_bridge gpio_dma_stream (
        .clk(wb_clk_i),
        .rst(wb_rst_i),
        .gpio_rx_valid(gpio_rx_valid),
        .gpio_rx_data(gpio_rx_data),
        .gpio_rx_ready(gpio_rx_ready),
        .gpio_tx_valid(gpio_tx_valid),
        .gpio_tx_data(gpio_tx_data),
        .gpio_tx_ready(gpio_tx_ready),
        .periph_rd_req_valid(periph_rd_req_valid),
        .periph_rd_req_addr(periph_rd_req_addr),
        .periph_rd_req_ready(periph_rd_req_ready),
        .periph_rd_rsp_valid(periph_rd_rsp_valid),
        .periph_rd_rsp_data(periph_rd_rsp_data),
        .periph_rd_rsp_ready(periph_rd_rsp_ready),
        .periph_wr_req_valid(periph_wr_req_valid),
        .periph_wr_req_addr(periph_wr_req_addr),
        .periph_wr_req_data(periph_wr_req_data),
        .periph_wr_req_strb(periph_wr_req_strb),
        .periph_wr_req_ready(periph_wr_req_ready)
    );

    wire [127:0] periph_debug = {
        periph_wr_req_data,
        periph_wr_req_addr,
        periph_rd_req_addr,
        20'd0,
        noc_node_valid,
        gpio_rx_ready,
        gpio_tx_valid,
        periph_wr_req_strb,
        periph_wr_req_valid,
        periph_rd_rsp_ready,
        periph_rd_req_valid
    };

    wire [`MPRJ_IO_PADS-1:0] gpio_tx_valid_vec = ({{(`MPRJ_IO_PADS-1){1'b0}}, gpio_tx_valid} << 28);
    wire [`MPRJ_IO_PADS-1:0] gpio_tx_data_vec = ({{(`MPRJ_IO_PADS-8){1'b0}}, gpio_tx_data} << 29);
    wire [`MPRJ_IO_PADS-1:0] gpio_rx_ready_vec = ({{(`MPRJ_IO_PADS-1){1'b0}}, gpio_rx_ready} << 37);
    wire [`MPRJ_IO_PADS-1:0] gpio_out_mask =
        ({{(`MPRJ_IO_PADS-1){1'b0}}, 1'b1} << 28) |
        ({{(`MPRJ_IO_PADS-8){1'b0}}, 8'hFF} << 29) |
        ({{(`MPRJ_IO_PADS-1){1'b0}}, 1'b1} << 37);

    assign io_out = gpio_tx_valid_vec | gpio_tx_data_vec | gpio_rx_ready_vec;
    assign io_oeb = ~gpio_out_mask;

    assign la_data_out = noc_flit_valid ? noc_flit_data : periph_debug;
    assign user_irq = {error_sticky, noc_flit_valid, result_valid};

`ifdef X1_ANALOG_WEIGHT_BEHAV
    localparam integer X1_ANALOG_WEIGHT_BEHAV_MODE = 1;
`else
    localparam integer X1_ANALOG_WEIGHT_BEHAV_MODE = 0;
`endif

    x1_memory_compiler_noc #(
        .NUM_MACROS(4),
        .NUM_NOC_NODES(4),
        .X1_ANALOG_WEIGHT_MODE(X1_ANALOG_WEIGHT_BEHAV_MODE)
    ) x1_mem_compiler (
`ifdef USE_POWER_PINS
        .VDDC1(vccd1),
        .VDDC2(vccd2),
        .VDDA1(vdda1),
        .VDDA2(vdda2),
        .VSS(vssd1),
`endif
        .wb_clk_i(wb_clk_i),
        .wb_rst_i(wb_rst_i),
        .wbs_stb_i(wbs_stb_i),
        .wbs_cyc_i(wbs_cyc_i),
        .wbs_we_i(wbs_we_i),
        .wbs_sel_i(wbs_sel_i),
        .wbs_dat_i(wbs_dat_i),
        .wbs_adr_i(wbs_adr_i),
        .wbs_ack_o(wbs_ack_o),
        .wbs_dat_o(wbs_dat_o),
        .ScanInCC(scan_in_cc),
        .ScanInDL(scan_in_dl),
        .ScanInDR(scan_in_dr),
        .TM(tm),
        .ScanOutCC(scan_out_cc),
        .Iref(analog_io[27]),
        .Vcc_read(analog_io[26]),
        .Vcomp(analog_io[25]),
        .Bias_comp2(analog_io[24]),
        .Vcc_wl_read(analog_io[19]),
        .Vcc_wl_set(analog_io[23]),
        .Vbias(analog_io[22]),
        .Vcc_wl_reset(analog_io[21]),
        .Vcc_set(analog_io[20]),
        .dc_bias(analog_io[18]),
        .noc_flit_data(noc_flit_data),
        .noc_flit_valid(noc_flit_valid),
        .noc_flit_last(noc_flit_last),
        .noc_flit_vc(noc_flit_vc),
        .noc_dst(noc_dst),
        .noc_node_valid(noc_node_valid),
        .noc_node_flit_data(noc_node_flit_data),
        .noc_node_vc(noc_node_vc),
        .noc_node_src(noc_node_src),
        .compiler_busy(compiler_busy),
        .init_done(init_done),
        .result_valid(result_valid),
        .error_sticky(error_sticky),
        .periph_rd_req_valid(periph_rd_req_valid),
        .periph_rd_req_addr(periph_rd_req_addr),
        .periph_rd_req_ready(periph_rd_req_ready),
        .periph_rd_rsp_valid(periph_rd_rsp_valid),
        .periph_rd_rsp_data(periph_rd_rsp_data),
        .periph_rd_rsp_ready(periph_rd_rsp_ready),
        .periph_wr_req_valid(periph_wr_req_valid),
        .periph_wr_req_addr(periph_wr_req_addr),
        .periph_wr_req_data(periph_wr_req_data),
        .periph_wr_req_strb(periph_wr_req_strb),
        .periph_wr_req_ready(periph_wr_req_ready)
    );

endmodule

module gpio_dma_stream_bridge (
    input         clk,
    input         rst,

    input         gpio_rx_valid,
    input  [7:0]  gpio_rx_data,
    output reg    gpio_rx_ready,
    output reg    gpio_tx_valid,
    output reg [7:0] gpio_tx_data,
    input         gpio_tx_ready,

    input         periph_rd_req_valid,
    input  [31:0] periph_rd_req_addr,
    output reg    periph_rd_req_ready,
    output reg    periph_rd_rsp_valid,
    output reg [31:0] periph_rd_rsp_data,
    input         periph_rd_rsp_ready,

    input         periph_wr_req_valid,
    input  [31:0] periph_wr_req_addr,
    input  [31:0] periph_wr_req_data,
    input  [3:0]  periph_wr_req_strb,
    output reg    periph_wr_req_ready
);
    localparam [3:0] G_IDLE = 4'd0;
    localparam [3:0] G_TX_READ = 4'd1;
    localparam [3:0] G_WAIT_READ_OPCODE = 4'd2;
    localparam [3:0] G_RX_READ_0 = 4'd3;
    localparam [3:0] G_RX_READ_1 = 4'd4;
    localparam [3:0] G_RX_READ_2 = 4'd5;
    localparam [3:0] G_RX_READ_3 = 4'd6;
    localparam [3:0] G_RSP_VALID = 4'd7;
    localparam [3:0] G_TX_WRITE = 4'd8;
    localparam [3:0] G_WR_ACK = 4'd9;

    localparam [7:0] FRAME_RD_REQ  = 8'h01;
    localparam [7:0] FRAME_WR_REQ  = 8'h02;
    localparam [7:0] FRAME_RD_RESP = 8'h81;

    reg [3:0] state;
    reg [3:0] tx_index;
    reg [31:0] latched_addr;
    reg [31:0] latched_wdata;
    reg [3:0] latched_strb;
    reg [31:0] rx_data_shift;

    function [7:0] read_req_byte;
        input [3:0] idx;
        input [31:0] addr;
        begin
            case (idx)
                4'd0: read_req_byte = FRAME_RD_REQ;
                4'd1: read_req_byte = addr[7:0];
                4'd2: read_req_byte = addr[15:8];
                4'd3: read_req_byte = addr[23:16];
                default: read_req_byte = addr[31:24];
            endcase
        end
    endfunction

    function [7:0] write_req_byte;
        input [3:0] idx;
        input [31:0] addr;
        input [31:0] data;
        input [3:0] strb;
        begin
            case (idx)
                4'd0: write_req_byte = FRAME_WR_REQ;
                4'd1: write_req_byte = addr[7:0];
                4'd2: write_req_byte = addr[15:8];
                4'd3: write_req_byte = addr[23:16];
                4'd4: write_req_byte = addr[31:24];
                4'd5: write_req_byte = data[7:0];
                4'd6: write_req_byte = data[15:8];
                4'd7: write_req_byte = data[23:16];
                4'd8: write_req_byte = data[31:24];
                default: write_req_byte = {4'd0, strb};
            endcase
        end
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= G_IDLE;
            tx_index <= 4'd0;
            latched_addr <= 32'd0;
            latched_wdata <= 32'd0;
            latched_strb <= 4'd0;
            rx_data_shift <= 32'd0;
            gpio_rx_ready <= 1'b0;
            gpio_tx_valid <= 1'b0;
            gpio_tx_data <= 8'd0;
            periph_rd_req_ready <= 1'b0;
            periph_rd_rsp_valid <= 1'b0;
            periph_rd_rsp_data <= 32'd0;
            periph_wr_req_ready <= 1'b0;
        end else begin
            periph_rd_req_ready <= 1'b0;
            periph_wr_req_ready <= 1'b0;
            gpio_rx_ready <= 1'b0;
            gpio_tx_valid <= 1'b0;
            gpio_tx_data <= 8'd0;

            if (periph_rd_rsp_valid && periph_rd_rsp_ready)
                periph_rd_rsp_valid <= 1'b0;

            case (state)
                G_IDLE: begin
                    if (periph_rd_req_valid) begin
                        latched_addr <= periph_rd_req_addr;
                        tx_index <= 4'd0;
                        state <= G_TX_READ;
                    end else if (periph_wr_req_valid) begin
                        latched_addr <= periph_wr_req_addr;
                        latched_wdata <= periph_wr_req_data;
                        latched_strb <= periph_wr_req_strb;
                        tx_index <= 4'd0;
                        state <= G_TX_WRITE;
                    end
                end

                G_TX_READ: begin
                    gpio_tx_valid <= 1'b1;
                    gpio_tx_data <= read_req_byte(tx_index, latched_addr);
                    if (gpio_tx_ready) begin
                        if (tx_index == 4'd4) begin
                            periph_rd_req_ready <= 1'b1;
                            tx_index <= 4'd0;
                            state <= G_WAIT_READ_OPCODE;
                        end else begin
                            tx_index <= tx_index + 4'd1;
                        end
                    end
                end

                G_WAIT_READ_OPCODE: begin
                    gpio_rx_ready <= 1'b1;
                    if (gpio_rx_valid && (gpio_rx_data == FRAME_RD_RESP)) begin
                        state <= G_RX_READ_0;
                    end
                end

                G_RX_READ_0: begin
                    gpio_rx_ready <= 1'b1;
                    if (gpio_rx_valid) begin
                        rx_data_shift[7:0] <= gpio_rx_data;
                        state <= G_RX_READ_1;
                    end
                end

                G_RX_READ_1: begin
                    gpio_rx_ready <= 1'b1;
                    if (gpio_rx_valid) begin
                        rx_data_shift[15:8] <= gpio_rx_data;
                        state <= G_RX_READ_2;
                    end
                end

                G_RX_READ_2: begin
                    gpio_rx_ready <= 1'b1;
                    if (gpio_rx_valid) begin
                        rx_data_shift[23:16] <= gpio_rx_data;
                        state <= G_RX_READ_3;
                    end
                end

                G_RX_READ_3: begin
                    gpio_rx_ready <= 1'b1;
                    if (gpio_rx_valid) begin
                        periph_rd_rsp_data <= {gpio_rx_data, rx_data_shift[23:0]};
                        periph_rd_rsp_valid <= 1'b1;
                        state <= G_RSP_VALID;
                    end
                end

                G_RSP_VALID: begin
                    if (!periph_rd_rsp_valid || periph_rd_rsp_ready) begin
                        state <= G_IDLE;
                    end
                end

                G_TX_WRITE: begin
                    gpio_tx_valid <= 1'b1;
                    gpio_tx_data <= write_req_byte(tx_index, latched_addr, latched_wdata, latched_strb);
                    if (gpio_tx_ready) begin
                        if (tx_index == 4'd9) begin
                            periph_wr_req_ready <= 1'b1;
                            tx_index <= 4'd0;
                            state <= G_WR_ACK;
                        end else begin
                            tx_index <= tx_index + 4'd1;
                        end
                    end
                end

                G_WR_ACK: begin
                    state <= G_IDLE;
                end

                default: begin
                    state <= G_IDLE;
                end
            endcase
        end
    end
endmodule
`default_nettype wire
