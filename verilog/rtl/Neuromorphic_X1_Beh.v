
`timescale 1ns / 1ps

`ifdef USE_POWER_PINS
    `define USE_PG_PIN
`endif

module Neuromorphic_X1_wb (

`ifdef USE_PG_PIN
   inout         VDDC1,            // 0 V analog ground
   inout         VDDC2,            // 0 V analog ground
   inout         VDDA1,           // 1.8 V analog supply (mapped to vdda1)
   inout         VDDA2,           // 1.8 V analog supply (mapped to vdda1)
   inout         VSS,           // 1.8 V analog core digital supply (mapped to vccd1)
`endif
  input         user_clk,     // user clock
  input         user_rst,     // user reset
  input         wb_clk_i,     // Wishbone clock
  input         wb_rst_i,     // Wishbone reset (Active High)
  input         wbs_stb_i,    // Wishbone strobe
  input         wbs_cyc_i,    // Wishbone cycle indicator
  input         wbs_we_i,     // Wishbone write enable: 1=write, 0=read
  input  [3:0]  wbs_sel_i,    // Wishbone byte select (must be 4'hF for 32-bit op)
  input  [31:0] wbs_dat_i,    // Wishbone write data (becomes DI to core)
  input  [31:0] wbs_adr_i,    // Wishbone address
  output [31:0] wbs_dat_o,    // Wishbone read data output (driven by DO from core)
  output        wbs_ack_o,     // Wishbone acknowledge output (ack_out from core)
  
  // Scan/Test Pins
  input         ScanInCC,        // Scan enable
  input         ScanInDL,        // Data scan chain input (user_clk domain)
  input         ScanInDR,        // Data scan chain input (wb_clk domain)
  input         TM,              // Test mode
  output        ScanOutCC,       // Data scan chain output

  // Analog Pins
  input         Iref,            // 100 µA current reference
  input         Vcc_read,        // 0.3 V read rail
  input         Vcomp,           // 0.6 V comparator bias
  input         Bias_comp2,      // 0.6 V comparator bias
  input         Vcc_wl_read,     // 0.7 V wordline read rail
  input         Vcc_wl_set,      // 1.8 V wordline set rail
  input         Vbias,           // 1.8 V analog bias
  input         Vcc_wl_reset,    // 2.6 V wordline reset rail
  input         Vcc_set,         // 3.3 V array set rail
  input         dc_bias
);

	parameter [31:0] ADDR_MATCH = 32'h3000_0004;
	
	// --------------------------------------------------------------------------
  // Internal wires connecting the shim to the behavioral core
  // --------------------------------------------------------------------------
	wire        CLKin;
  wire        RSTin;
  wire        EN;
  wire [31:0] DI;
  wire        W_RB;
  wire [31:0] DO;
  wire        ack_out;
	
	// Map WB to core
	assign EN = (wbs_stb_i && wbs_cyc_i && (wbs_adr_i == ADDR_MATCH) && (wbs_sel_i == 4'hF));
	assign CLKin      = wb_clk_i;
  assign RSTin      = wb_rst_i;
	assign DI         = wbs_dat_i;
	assign W_RB       = wbs_we_i;
	assign wbs_dat_o  = DO;
	assign wbs_ack_o  = ack_out;
	
	// Instantiate the behavioral core
	Neuromorphic_X1_beh core_inst (
	`ifdef USE_PG_PIN
      .VDDC1(VDDC1),
      .VDDC2(VDDC2),
      .VDDA1(VDDA1),
      .VDDA2(VDDA2),
      .VSS(VSS),
`endif
    .CLKin      (CLKin),
    .RSTin      (RSTin),
    .EN         (EN),
    .DI         (DI),
    .W_RB       (W_RB),
    .DO         (DO),
    .ack_out   (ack_out),
    
    // Scan/Test Pins
    .ScanInCC(ScanInCC),
    .ScanInDL(ScanInDL),
    .ScanInDR(ScanInDR),
    .TM(TM),
    .ScanOutCC(ScanOutCC),

    // Analog Pins
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
	
endmodule


// -----------------------------------------------------------------------------
// X1 behavioral core (sim only)
//  - 32x32 1-bit X1 cell array
//  - first three writes capture RTL timing/threshold config packets
//  - MODE=00: delayed per-cell reset to 0
//  - MODE=01: delayed single-cell read, returned through output FIFO
//  - MODE=10: compute mode; collect three packets, then return a zero-extended
//              19-bit TDC/scratchpad-style word through the output FIFO
//  - MODE=11: delayed per-cell set/program using DATA[7:0] threshold
//  - WB READ ACKs only when result data is ready
// -----------------------------------------------------------------------------


module Neuromorphic_X1_beh (

`ifdef USE_PG_PIN
   inout         VDDC1,            // 0 V analog ground
   inout         VDDC2,            // 0 V analog ground
   inout         VDDA1,           // 1.8 V analog supply (mapped to vdda1)
   inout         VDDA2,           // 1.8 V analog supply (mapped to vdda1)
   inout         VSS,           // 1.8 V analog core digital supply (mapped to vccd1)
`endif

  input         CLKin,
	input         RSTin,
	input         EN,
	input  [31:0] DI,
	input         W_RB,
	output reg [31:0] DO,
	output reg    ack_out,
	
	// Scan/Test Pins
  input         ScanInCC,        // Scan enable
  input         ScanInDL,        // Scan data in (user_clk domain)
  input         ScanInDR,        // Scan data in (wb_clk domain)
  input         TM,              // Test mode
  output        ScanOutCC,       // Scan data out

  // Analog Pins
  input         Iref,            // 100 µA current reference
  input         Vcc_read,        // 0.3 V read rail
  input         Vcomp,           // 0.6 V comparator bias
  input         Bias_comp2,      // 0.6 V comparator bias
  input         Vcc_wl_read,     // 0.7 V wordline read rail
  input         Vcc_wl_set,      // 1.8 V wordline set rail
  input         Vbias,           // 1.8 V analog bias
  input         Vcc_wl_reset,    // 2.6 V wordline reset rail
  input         Vcc_set,         // 3.3 V set rail
  input         dc_bias
);
  
  assign ScanOutCC = TM ? ScanInDR : 1'b0;

  // Delay model note:
  // The old submit behavioral model used fixed RD_Dly=44 and WR_Dly=200.
  // These values are now tied back to the exported RTL controller counters.
  // The read path has two 64-cycle analog settle waits, a 20-cycle
  // subtractor/TDC window, and a small read-done retry window: 154 cycles.
  // Cell set/reset below are selected-cell program operations, not system
  // reset/set. They add the RTL program PWM loop, so the default set/reset
  // latency is 5 + ((3 + 1) * 12) + RD_Dly = 207 cycles.
  // The real RTL has async WB/user FIFOs and rd_vld timing; this remains a
  // single-clock behavioral approximation. RTL_RDVLD_CYCLES adds the one-cycle
  // scratchpad/read-FIFO valid delay before data is visible to WB.
  parameter integer RTL_SUB_DLY           = 64;
  parameter integer RTL_SUBTRACT_CYCLES   = 20;
  parameter integer RTL_READ_DONE_RETRIES = 6;
  parameter integer RTL_PGM_SETUP_CYCLES  = 5;
  parameter integer RTL_PGM_LOOP_CYCLES   = 12;
  parameter integer RTL_RDVLD_CYCLES      = 1;
  parameter integer CFG_Dly               = 3;
  parameter integer RD_Dly                = (2 * RTL_SUB_DLY) + RTL_SUBTRACT_CYCLES + RTL_READ_DONE_RETRIES;
  parameter integer COMPUTE_Dly           = RD_Dly;
  parameter integer WR_Dly                = RTL_PGM_SETUP_CYCLES + ((10'd3 + 10'd1) * RTL_PGM_LOOP_CYCLES) + RD_Dly;
  parameter integer RESET_Dly             = WR_Dly;
  parameter [31:0] EMPTY_TOKEN            = 32'hDEAD_C0DE;

  // ---------------------------------------------------------------------------
  // Internals
  // ---------------------------------------------------------------------------
  integer r, c, k, m;                          // loop indices for init and delays

  // 32x32 memory array (row = [29:25], col = [24:20])
  reg array_mem [0:31][0:31];  // 32x32 memory array (1-bit values)

  // RTL-style configuration registers loaded by the first three Wishbone writes.
  reg [1:0]  config_pkt_count;
  reg [15:0] target_set1;
  reg [15:0] target_set2;
  reg [15:0] target_reset1;
  reg [15:0] target_reset2;
  reg [9:0]  no_clk_cycles;
  reg [9:0]  counter_value;
  reg [6:0]  tdc_time_out;
  reg [1:0]  tdc_dead_time;

  // Compute mode collects exactly three packets before producing one result.
  // Actual RTL readback is {13'b0, scratchpad_data_out[18:0]}, not a 32-bit
  // Boolean column vector.
  reg [1:0]  compute_pkt_count;
  reg [31:0] compute_row_mask;
  reg [31:0] compute_col_mask;
  reg        compute_full_row;
  reg [7:0]  compute_pwm0;
  reg [7:0]  compute_pwm1;
  reg [7:0]  compute_pwm2;

  // Two 32-deep FIFOs (behavioral)
  reg [31:0] ip_fifo [0:31];                   // WB -> Engine commands
  reg [31:0] op_fifo [0:31];                   // Engine -> WB results

  // --- 5-bit pointers + separate wrap flags (preserves original semantics) ---
  // Input FIFO (WB producer / Engine consumer)
  reg  [4:0] ip_wptr_idx;   // producer index (WB)
  reg        ip_wptr_wrap;  // producer wrap bit
  reg  [4:0] ip_rptr_idx;   // consumer index (Engine)
  reg        ip_rptr_wrap;  // consumer wrap bit

  // Output FIFO (Engine producer / WB consumer)
  reg  [4:0] op_wptr_idx;   // producer index (Engine)
  reg        op_wptr_wrap;  // producer wrap bit
  reg  [4:0] op_rptr_idx;   // consumer index (WB)
  reg        op_rptr_wrap;  // consumer wrap bit

  // FIFO status using index + wrap flags
  wire ip_empty = (ip_wptr_idx == ip_rptr_idx) && (ip_wptr_wrap == ip_rptr_wrap);
  wire ip_full  = (ip_wptr_idx == ip_rptr_idx) && (ip_wptr_wrap != ip_rptr_wrap);

  wire op_empty = (op_wptr_idx == op_rptr_idx) && (op_wptr_wrap == op_rptr_wrap);
  wire op_full  = (op_wptr_idx == op_rptr_idx) && (op_wptr_wrap != op_rptr_wrap);
	
	// Next index helpers
  wire [4:0] ip_wptr_idx_next = (ip_wptr_idx == 5'd31) ? 5'd0 : (ip_wptr_idx + 5'd1);
  wire       ip_wptr_wrap_next = (ip_wptr_idx == 5'd31) ? ~ip_wptr_wrap : ip_wptr_wrap;

  wire [4:0] ip_rptr_idx_next = (ip_rptr_idx == 5'd31) ? 5'd0 : (ip_rptr_idx + 5'd1);
  wire       ip_rptr_wrap_next = (ip_rptr_idx == 5'd31) ? ~ip_rptr_wrap : ip_rptr_wrap;

  wire [4:0] op_wptr_idx_next = (op_wptr_idx == 5'd31) ? 5'd0 : (op_wptr_idx + 5'd1);
  wire       op_wptr_wrap_next = (op_wptr_idx == 5'd31) ? ~op_wptr_wrap : op_wptr_wrap;

  wire [4:0] op_rptr_idx_next = (op_rptr_idx == 5'd31) ? 5'd0 : (op_rptr_idx + 5'd1);
  wire       op_rptr_wrap_next = (op_rptr_idx == 5'd31) ? ~op_rptr_wrap : op_rptr_wrap;

  // Engine state
  reg        in_process;                        // engine busy flag
  reg [31:0] DI_local;                          // latched command
  reg [31:0] DO_local;                          // latched read data

  function [31:0] onehot32;
    input [4:0] idx;
    begin
      onehot32 = 32'b0;
      onehot32[idx] = 1'b1;
    end
  endfunction

  function [18:0] compute_tdc_scratchpad_word;
    input [31:0] row_mask;
    input [31:0] col_mask;
    integer rr, cc;
    reg [4:0] first_col;
    reg       first_col_seen;
    reg [8:0] hit_count;
    begin
      first_col      = 5'd0;
      first_col_seen = 1'b0;
      hit_count      = 9'd0;
      for (cc = 0; cc < 32; cc = cc + 1) begin
        if (col_mask[cc]) begin
          if (!first_col_seen) begin
            first_col      = cc[4:0];
            first_col_seen = 1'b1;
          end
          for (rr = 0; rr < 32; rr = rr + 1) begin
            if (row_mask[rr] && array_mem[rr][cc])
              hit_count = hit_count + 9'd1;
          end
        end
      end
      compute_tdc_scratchpad_word = {first_col, 5'd0, hit_count};
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Wishbone side (behavioral, decoupled from engine)
  // ---------------------------------------------------------------------------
  always @(posedge CLKin or posedge RSTin) begin
    if (RSTin) begin
      DO          <= 32'd0;
      ack_out     <= 1'b0;
      ip_wptr_idx <= 5'd0;
      ip_wptr_wrap<= 1'b0;
      op_rptr_idx <= 5'd0;
      op_rptr_wrap<= 1'b0;
    end else begin
      ack_out <= 1'b0;
      // WRITE request -> push to ip_fifo if not full
      if (EN && W_RB && !ack_out) begin
        if (!ip_full) begin
          ack_out <= 1'b1;
          ip_fifo[ip_wptr_idx] <= DI;
          ip_wptr_idx  <= ip_wptr_idx_next;
          ip_wptr_wrap <= ip_wptr_wrap_next;
        end
      end
      // READ request -> pop from op_fifo only when data is ready.
      else if (EN && !W_RB && !ack_out) begin
        if (!op_empty) begin
          ack_out <= 1'b1;
          DO      <= op_fifo[op_rptr_idx];
          op_rptr_idx  <= op_rptr_idx_next;
          op_rptr_wrap <= op_rptr_wrap_next;
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Engine side (simulation-only)
  // ---------------------------------------------------------------------------
  always @(posedge CLKin or posedge RSTin) begin
    if (RSTin) begin
      in_process        <= 1'b0;
      ip_rptr_idx       <= 5'd0;
      ip_rptr_wrap      <= 1'b0;
      op_wptr_idx       <= 5'd0;
      op_wptr_wrap      <= 1'b0;
      config_pkt_count  <= 2'd0;
      compute_pkt_count <= 2'd0;
      compute_row_mask  <= 32'd0;
      compute_col_mask  <= 32'd0;
      compute_full_row  <= 1'b0;
      compute_pwm0      <= 8'd0;
      compute_pwm1      <= 8'd0;
      compute_pwm2      <= 8'd0;
      target_set1       <= 16'hc40f;
      target_set2       <= 16'ha203;
      target_reset1     <= 16'h0d43;
      target_reset2     <= 16'h0f03;
      no_clk_cycles     <= 10'd3;
      counter_value     <= 10'd3;
      tdc_time_out      <= 7'd32;
      tdc_dead_time     <= 2'b01;
			
    end else begin
      if (!in_process) begin
        if (!ip_empty) begin
          in_process <= 1'b1;
          DI_local   = ip_fifo[ip_rptr_idx]; // latch command

          // -------- CONFIG PACKETS: first three writes after reset --------
          if (config_pkt_count < 2'd3) begin
            for (k = 0; k < CFG_Dly; k = k + 1) @(posedge CLKin);
            case (config_pkt_count)
              2'd0: begin
                target_set1 <= DI_local[15:0];
                target_set2 <= DI_local[31:16];
              end
              2'd1: begin
                target_reset1 <= DI_local[15:0];
                target_reset2 <= DI_local[31:16];
              end
              default: begin
                no_clk_cycles <= DI_local[9:0];
                counter_value <= DI_local[19:10];
                tdc_time_out  <= DI_local[26:20];
                tdc_dead_time <= DI_local[31:30];
              end
            endcase

            config_pkt_count <= config_pkt_count + 1'b1;
            ip_rptr_idx      <= ip_rptr_idx_next;
            ip_rptr_wrap     <= ip_rptr_wrap_next;
            in_process       <= 1'b0;
          end

          // ---------------- CELL SET / PROGRAM OP (MODE=2'b11) ----
          else if (DI_local[31:30] == 2'b11) begin
            compute_pkt_count <= 2'd0;
            compute_row_mask  <= 32'd0;
            compute_col_mask  <= 32'd0;
            compute_full_row  <= 1'b0;
            // Selected-cell set path. The RTL drives one row/col with PWM,
            // then verifies through the read/TDC path; this 1-bit model stores
            // the final selected-cell state after the equivalent delay.
					  for (k = 0; k < (RTL_PGM_SETUP_CYCLES + ((no_clk_cycles + 1) * RTL_PGM_LOOP_CYCLES) + RD_Dly); k = k + 1) @(posedge CLKin);
            array_mem[DI_local[29:25]][DI_local[24:20]] = (DI_local[7:0] > 8'h7F);

            ip_rptr_idx  <= ip_rptr_idx_next;
            ip_rptr_wrap <= ip_rptr_wrap_next;
            in_process   <= 1'b0;
          end

          // --------------- CELL RESET OP (MODE=2'b00) -------------
          else if (DI_local[31:30] == 2'b00) begin
            compute_pkt_count <= 2'd0;
            compute_row_mask  <= 32'd0;
            compute_col_mask  <= 32'd0;
            compute_full_row  <= 1'b0;
            // Selected-cell reset path, paired with cell set timing above.
            // This is not RSTin/Wishbone/CPU reset; only row[29:25], col[24:20]
            // is cleared after the program-reset style delay.
					  for (k = 0; k < (RTL_PGM_SETUP_CYCLES + ((no_clk_cycles + 1) * RTL_PGM_LOOP_CYCLES) + RD_Dly); k = k + 1) @(posedge CLKin);
            array_mem[DI_local[29:25]][DI_local[24:20]] = 1'b0;

            ip_rptr_idx  <= ip_rptr_idx_next;
            ip_rptr_wrap <= ip_rptr_wrap_next;
            in_process   <= 1'b0;
          end

          // ---------------- READ OP (MODE=2'b01) -----------------
          else if (DI_local[31:30] == 2'b01) begin
            compute_pkt_count <= 2'd0;
            compute_row_mask  <= 32'd0;
            compute_col_mask  <= 32'd0;
            compute_full_row  <= 1'b0;
            if (op_full) begin
              in_process <= 1'b0;
            end else begin
              for (m = 0; m < (RD_Dly + RTL_RDVLD_CYCLES); m = m + 1) @(posedge CLKin);
              DO_local = {31'b0, array_mem[DI_local[29:25]][DI_local[24:20]]};
              op_fifo[op_wptr_idx] <= DO_local;
              op_wptr_idx  <= op_wptr_idx_next;
              op_wptr_wrap <= op_wptr_wrap_next;
              ip_rptr_idx  <= ip_rptr_idx_next;
              ip_rptr_wrap <= ip_rptr_wrap_next;
              in_process   <= 1'b0;
            end
          end

          // ---------------- COMPUTE OP (MODE=2'b10) ----------------
          else if (DI_local[31:30] == 2'b10) begin
            if (compute_pkt_count == 2'd0) begin
              compute_row_mask  <= onehot32(DI_local[29:25]);
              compute_col_mask  <= onehot32(DI_local[24:20]);
              compute_full_row  <= DI_local[18];
              compute_pwm0      <= DI_local[7:0];
              compute_pkt_count <= 2'd1;
              ip_rptr_idx       <= ip_rptr_idx_next;
              ip_rptr_wrap      <= ip_rptr_wrap_next;
              in_process        <= 1'b0;
            end else if (compute_pkt_count == 2'd1) begin
              compute_row_mask  <= compute_row_mask | onehot32(DI_local[29:25]);
              compute_col_mask  <= compute_col_mask | onehot32(DI_local[24:20]);
              compute_full_row  <= compute_full_row | DI_local[18];
              compute_pwm1      <= DI_local[7:0];
              compute_pkt_count <= 2'd2;
              ip_rptr_idx       <= ip_rptr_idx_next;
              ip_rptr_wrap      <= ip_rptr_wrap_next;
              in_process        <= 1'b0;
            end else if (op_full) begin
              in_process <= 1'b0;
            end else begin
              compute_pwm2 = DI_local[7:0];
              for (m = 0; m < (COMPUTE_Dly + RTL_RDVLD_CYCLES); m = m + 1) @(posedge CLKin);
              DO_local = {13'b0, compute_tdc_scratchpad_word(
                compute_row_mask | onehot32(DI_local[29:25]),
                (compute_full_row | DI_local[18]) ? 32'hFFFF_FFFF : (compute_col_mask | onehot32(DI_local[24:20]))
              )};
              op_fifo[op_wptr_idx] <= DO_local;
              op_wptr_idx       <= op_wptr_idx_next;
              op_wptr_wrap      <= op_wptr_wrap_next;
              compute_pkt_count <= 2'd0;
              compute_row_mask  <= 32'd0;
              compute_col_mask  <= 32'd0;
              compute_full_row  <= 1'b0;
              ip_rptr_idx       <= ip_rptr_idx_next;
              ip_rptr_wrap      <= ip_rptr_wrap_next;
              in_process        <= 1'b0;
            end
          end

          // --------------- UNKNOWN OPCODE: drop it ----------------
          else begin
            compute_pkt_count <= 2'd0;
            compute_row_mask  <= 32'd0;
            compute_col_mask  <= 32'd0;
            compute_full_row  <= 1'b0;
            ip_rptr_idx       <= ip_rptr_idx_next;
            ip_rptr_wrap      <= ip_rptr_wrap_next;
            in_process        <= 1'b0;
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Init memory to 0 (sim-only convenience)
  // ---------------------------------------------------------------------------
  initial begin
    for (r = 0; r < 32; r = r + 1) begin
      for (c = 0; c < 32; c = c + 1) begin
        array_mem[r][c] = 1'b0;
      end
    end		
  end

endmodule
