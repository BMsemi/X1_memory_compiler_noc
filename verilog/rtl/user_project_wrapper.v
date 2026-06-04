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

    wire scan_in_cc = io_in[35];
    wire scan_in_dl = io_in[22];
    wire scan_in_dr = io_in[21];
    wire tm         = io_in[36];
    wire scan_out_cc;

    wire [127:0] noc_flit_data;
    wire         noc_flit_valid;
    wire         noc_flit_last;
    wire [2:0]   noc_flit_vc;
    wire [3:0]   noc_dst;
    wire         compiler_busy;
    wire         init_done;
    wire         result_valid;
    wire         error_sticky;

    wire [`MPRJ_IO_PADS-1:0] scan_out_vec = ({{(`MPRJ_IO_PADS-1){1'b0}}, scan_out_cc} << 23);
    wire [`MPRJ_IO_PADS-1:0] noc_valid_vec = ({{(`MPRJ_IO_PADS-1){1'b0}}, noc_flit_valid} << 10);
    wire [`MPRJ_IO_PADS-1:0] busy_vec = ({{(`MPRJ_IO_PADS-1){1'b0}}, compiler_busy} << 11);
    wire [`MPRJ_IO_PADS-1:0] init_vec = ({{(`MPRJ_IO_PADS-1){1'b0}}, init_done} << 12);
    wire [`MPRJ_IO_PADS-1:0] err_vec = ({{(`MPRJ_IO_PADS-1){1'b0}}, error_sticky} << 13);
    wire [`MPRJ_IO_PADS-1:0] debug_oeb_mask =
        ({{(`MPRJ_IO_PADS-1){1'b0}}, 1'b1} << 23) |
        ({{(`MPRJ_IO_PADS-1){1'b0}}, 1'b1} << 10) |
        ({{(`MPRJ_IO_PADS-1){1'b0}}, 1'b1} << 11) |
        ({{(`MPRJ_IO_PADS-1){1'b0}}, 1'b1} << 12) |
        ({{(`MPRJ_IO_PADS-1){1'b0}}, 1'b1} << 13);

    assign io_out = scan_out_vec | noc_valid_vec | busy_vec | init_vec | err_vec;
    assign io_oeb = ~debug_oeb_mask;

    assign la_data_out = noc_flit_data;
    assign user_irq = {error_sticky, noc_flit_valid, result_valid};

    x1_memory_compiler_noc #(
        .NUM_MACROS(4)
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
        .compiler_busy(compiler_busy),
        .init_done(init_done),
        .result_valid(result_valid),
        .error_sticky(error_sticky)
    );

endmodule
`default_nettype wire
