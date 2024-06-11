// Copyright (c) 2021 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

module DMA_ENGINE
#(
    parameter DW                = 32,   // data size
    parameter SA_WIDTH          = 4,    // systolic array width in PE count
    parameter BUF_AW            = 6,    // buffer address width
    parameter BUF_DW            = 128   // buffer data width
 )
(
    input   wire                clk,
    input   wire                rst_n,  // _n means active low

    // configuration registers
    input   wire    [31:0]      mat_a_addr_i,
    input   wire    [31:0]      mat_b_addr_i,
    input   wire    [31:0]      mat_c_addr_i,
    input   wire    [7:0]       mat_width_i,
    input   wire                start_i,
    output  reg                 done_o,

    // AMBA AXI interface
    AXI_AW_CH.master            axi_aw_if,
    AXI_W_CH.master             axi_w_if,
    AXI_B_CH.slave              axi_b_if,
    AXI_AR_CH.master            axi_ar_if,
    AXI_R_CH.slave              axi_r_if,

    // buffer interface
    output  reg                 buf_a_wren_o,
    output  wire   [BUF_AW-1:0] buf_a_waddr_o,
    output  reg    [BUF_DW/8-1:0] buf_a_wbyteenable_o,
    output  wire   [BUF_DW-1:0] buf_a_wdata_o,
    output  reg                 buf_b_wren_o,
    output  wire   [BUF_AW-1:0] buf_b_waddr_o,
    output  reg    [BUF_DW/8-1:0] buf_b_wbyteenable_o,
    output  wire   [BUF_DW-1:0] buf_b_wdata_o,

    // other module start
    output  reg                 mm_start_o,
    input   wire                mm_done_i,
    input   signed [2*DW:0]     accum_i[SA_WIDTH][SA_WIDTH]
);

    // TODO: implement me

endmodule
