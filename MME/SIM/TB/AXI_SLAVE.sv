// Copyright (c) 2021 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

`include "../../RTL/AXI_TYPEDEF.svh"

module AXI_SLAVE
#(
  parameter ADDR_WIDTH        = 16,
  parameter DATA_WIDTH        = `AXI_DATA_WIDTH,
  parameter ID_WIDTH          = `AXI_ID_WIDTH,
  parameter AWREADY_DELAY     = 1,
  parameter ARREADY_DELAY     = 1,
  parameter AR2R_DELAY        = 50
)
(
  input   wire                clk,
  input   wire                rst_n,  // _n means active low

  AXI_AW_CH                   aw_ch,
  AXI_W_CH                    w_ch,
  AXI_B_CH                    b_ch,
  AXI_AR_CH                   ar_ch,
  AXI_R_CH                    r_ch
);
  localparam  FIFO_DEPTH_LG2  = 4;
  localparam  DATA_DEPTH      = 1<<ADDR_WIDTH;

  logic   [7:0]               mem[DATA_DEPTH];

  function void write_byte(int addr, input bit [7:0] wdata);
    mem[addr]               = wdata;
  endfunction

  function void write_word(int addr, input bit [31:0] wdata);
    for (int i=0; i<4; i++) begin
      write_byte(addr+i, wdata[8*i +: 8]);    // [i*8+7:i*8]
    end
  endfunction

  function bit [7:0] read_byte(int addr);
    read_byte               = mem[addr];
  endfunction

  function bit [31:0] read_word(int addr);
    for (int i=0; i<4; i++) begin
      read_word[8*i +: 8] = read_byte(addr+i);// [i*8+7:i*8]
    end
  endfunction

  //----------------------------------------------------------
  // write channel
  //----------------------------------------------------------
  // AW
  wire  [ADDR_WIDTH-1:0]  aw2fifo_waddr_wdata;
  wire  [ID_WIDTH-1:0]    aw2fifo_wid_wdata;
  wire  [3:0]             aw2fifo_wlen_wdata;
  wire                    aw2wfifo_full, aw2wfifo_wren;

  // AW2W FIFO
  wire aw2w_waddr_full, aw2w_wid_full, aw2w_wlen_full;
  wire aw2w_waddr_wren, aw2w_wid_wren, aw2w_wlen_wren;
  wire aw2w_waddr_empty, aw2w_wid_empty, aw2w_wlen_empty;
  wire aw2w_waddr_rden, aw2w_wid_rden, aw2w_wlen_rden;

  // W
  wire                    aw2wfifo_empty, aw2wfifo_rden;
  wire  [ADDR_WIDTH-1:0]  aw2w_waddr_rdata;
  wire  [ID_WIDTH-1:0]    aw2w_wid_rdata;
  wire  [3:0]             aw2w_wlen_rdata;
  wire  [ADDR_WIDTH-1:0]  waddr;
  wire                    w2bfifo_full, w2bfifo_wren;

  // B
  wire  [ID_WIDTH-1:0]  w2b_bid_rdata;
  wire                  w2bfifo_empty, w2bfifo_rden;

  ////// AW channel //////
  AXI_SLAVE_AW  #(
    .ADDR_WIDTH             (ADDR_WIDTH),
    .DATA_WIDTH             (DATA_WIDTH),
    .ID_WIDTH               (ID_WIDTH),
    .AWREADY_DELAY          (AWREADY_DELAY)
  )   AW_CH_FSM   (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .awvalid_i              (aw_ch.awvalid),
    .awid_i                 (aw_ch.awid),
    .awaddr_i               (aw_ch.awaddr[ADDR_WIDTH-1:0]),
    .awlen_i                (aw_ch.awlen),
    .awsize_i               (aw_ch.awsize),
    .awburst_i              (aw_ch.awburst),
    .awready_o              (aw_ch.awready),

    .waddr_o                (aw2fifo_waddr_wdata),
    .wid_o                  (aw2fifo_wid_wdata),
    .wlen_o                 (aw2fifo_wlen_wdata),

    .full_i                 (aw2wfifo_full),
    .wren_o                 (aw2wfifo_wren)
  );

  ////// AW2W FIFO  //////
  assign aw2wfifo_full = aw2w_waddr_full | aw2w_wid_full | aw2w_wlen_full;
  assign aw2w_waddr_wren  = aw2wfifo_wren;
  assign aw2w_wid_wren    = aw2wfifo_wren;
  assign aw2w_wlen_wren   = aw2wfifo_wren;
  FIFO  #(
    .DEPTH_LG2          (FIFO_DEPTH_LG2),
    .DATA_WIDTH         (ADDR_WIDTH)
  )   AW2W_WADDR_FIFO  (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .full_o                 (aw2w_waddr_full),
    .wren_i                 (aw2w_waddr_wren),
    .wdata_i                (aw2fifo_waddr_wdata),

    .empty_o                (aw2w_waddr_empty),
    .rden_i                 (aw2w_waddr_rden),
    .rdata_o                (aw2w_waddr_rdata)
  );
  FIFO  #(
    .DEPTH_LG2          (FIFO_DEPTH_LG2),
    .DATA_WIDTH         (ID_WIDTH)
  )   AW2W_WID_FIFO  (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .full_o                 (aw2w_wid_full),
    .wren_i                 (aw2w_wid_wren),
    .wdata_i                (aw2fifo_wid_wdata),

    .empty_o                (aw2w_wid_empty),
    .rden_i                 (aw2w_wid_rden),
    .rdata_o                (aw2w_wid_rdata)
  );
  FIFO  #(
    .DEPTH_LG2          (FIFO_DEPTH_LG2),
    .DATA_WIDTH         (4)
  )   AW2W_WLEN_FIFO  (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .full_o                 (aw2w_wlen_full),
    .wren_i                 (aw2w_wlen_wren),
    .wdata_i                (aw2fifo_wlen_wdata),

    .empty_o                (aw2w_wlen_empty),
    .rden_i                 (aw2w_wlen_rden),
    .rdata_o                (aw2w_wlen_rdata)
  );

  ////// W channel  //////
  assign aw2wfifo_empty = aw2w_waddr_empty | aw2w_wlen_empty | aw2w_wid_empty;
  assign aw2w_waddr_rden  = aw2wfifo_rden;
  assign aw2w_wid_rden    = aw2wfifo_rden;
  assign aw2w_wlen_rden   = aw2wfifo_rden;

  AXI_SLAVE_W   #(
    .ADDR_WIDTH             (ADDR_WIDTH),
    .DATA_WIDTH             (DATA_WIDTH),
    .ID_WIDTH               (ID_WIDTH)
  )   W_CH_FSM  (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .wvalid_i               (w_ch.wvalid),
    .wid_i                  (w_ch.wid),
//    .wdata_i                (w_ch.wdata),
    .wstrb_i                (w_ch.wstrb),
    .wlast_i                (w_ch.wlast),
    .wready_o               (w_ch.wready),

    .awaddr_i               (aw2w_waddr_rdata),
    .awid_i                 (aw2w_wid_rdata),
    .awlen_i                (aw2w_wlen_rdata),

    .waddr_o                (waddr),
    
    .empty_i                (aw2wfifo_empty),
    .rden_o                 (aw2wfifo_rden),

    .full_i                 (w2bfifo_full),
    .wren_o                 (w2bfifo_wren)
  );
  always @(*) begin
    if (w_ch.wvalid) begin
      for (int i=0; i<DATA_WIDTH/8; i++) begin
        write_byte(waddr + i, w_ch.wdata[i*8 +: 8]);    // [i*8+7:i*8]
      end
    end
  end

  //////  W2B FIFO  //////
  FIFO  #(
    .DEPTH_LG2          (FIFO_DEPTH_LG2),
    .DATA_WIDTH         (ID_WIDTH)
  )   W2B_BID_FIFO  (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .full_o                 (w2bfifo_full),
    .wren_i                 (w2bfifo_wren),
    .wdata_i                (w_ch.wid),

    .empty_o                (w2bfifo_empty),
    .rden_i                 (w2bfifo_rden),
    .rdata_o                (w2b_bid_rdata)
  );

  ////// B channel  //////
  AXI_SLAVE_B   #(
    .ID_WIDTH                 (ID_WIDTH)
  )   B_CH_FSM  (
    .clk                      (clk),
    .rst_n                    (rst_n),

    .bvalid_o                 (b_ch.bvalid),
    .bid_o                    (b_ch.bid),
    .bresp_o                  (b_ch.bresp),
    .bready_i                 (b_ch.bready),

    .bid_i                    (w2b_bid_rdata),

    .empty_i                  (w2bfifo_empty),
    .rden_o                   (w2bfifo_rden)
  );

  //----------------------------------------------------------
  // read channel
  //----------------------------------------------------------
  // AR
  wire  [ADDR_WIDTH-1:0]  ar2fifo_raddr_wdata;
  wire  [ID_WIDTH-1:0]    ar2fifo_rid_wdata;
  wire  [3:0]             ar2fifo_rlen_wdata;
  wire                    ar2rfifo_full, ar2rfifo_wren;

  // AR2R FIFO
  wire ar2r_raddr_full, ar2r_rid_full, ar2r_rlen_full;
  wire ar2r_raddr_wren, ar2r_rid_wren, ar2r_rlen_wren;
  wire ar2r_raddr_empty, ar2r_rid_empty, ar2r_rlen_empty;
  wire ar2r_raddr_rden, ar2r_rid_rden, ar2r_rlen_rden;

  // R
  wire                    ar2rfifo_rden, ar2rfifo_empty;
  wire  [ADDR_WIDTH-1:0]  ar2r_raddr_rdata;
  wire  [ID_WIDTH-1:0]    ar2r_rid_rdata;
  wire  [3:0]             ar2r_rlen_rdata;
  wire  [ADDR_WIDTH-1:0]  raddr;

  ////// AR channel //////
  AXI_SLAVE_AR  #(
    .ADDR_WIDTH             (ADDR_WIDTH),
    .DATA_WIDTH             (DATA_WIDTH),
    .ID_WIDTH               (ID_WIDTH),
    .ARREADY_DELAY          (ARREADY_DELAY),
    .AR2R_DELAY             (AR2R_DELAY)
  )   AR_CH_FSM   (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .arvalid_i              (ar_ch.arvalid),
    .arid_i                 (ar_ch.arid),
    .araddr_i               (ar_ch.araddr[ADDR_WIDTH-1:0]),
    .arlen_i                (ar_ch.arlen),
    .arsize_i               (ar_ch.arsize),
    .arburst_i              (ar_ch.arburst),
    .arready_o              (ar_ch.arready),

    .raddr_o                (ar2fifo_raddr_wdata),
    .rid_o                  (ar2fifo_rid_wdata),
    .rlen_o                 (ar2fifo_rlen_wdata),

    .full_i                 (ar2rfifo_full),
    .wren_o                 (ar2rfifo_wren)
  );

  ////// AR2R FIFO  //////
  assign ar2rfifo_full = ar2r_raddr_full | ar2r_rid_full | ar2r_rlen_full;
  assign ar2r_raddr_wren  = ar2rfifo_wren;
  assign ar2r_rid_wren    = ar2rfifo_wren;
  assign ar2r_rlen_wren   = ar2rfifo_wren;
  FIFO  #(
    .DEPTH_LG2          (FIFO_DEPTH_LG2),
    .DATA_WIDTH         (ADDR_WIDTH)
  )   AR2R_RADDR_FIFO  (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .full_o                 (ar2r_raddr_full),
    .wren_i                 (ar2r_raddr_wren),
    .wdata_i                (ar2fifo_raddr_wdata),

    .empty_o                (ar2r_raddr_empty),
    .rden_i                 (ar2r_raddr_rden),
    .rdata_o                (ar2r_raddr_rdata)
  );
  FIFO  #(
    .DEPTH_LG2          (FIFO_DEPTH_LG2),
    .DATA_WIDTH         (ID_WIDTH)
  )   AR2R_RID_FIFO  (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .full_o                 (ar2r_rid_full),
    .wren_i                 (ar2r_rid_wren),
    .wdata_i                (ar2fifo_rid_wdata),

    .empty_o                (ar2r_rid_empty),
    .rden_i                 (ar2r_rid_rden),
    .rdata_o                (ar2r_rid_rdata)
  );
  FIFO  #(
    .DEPTH_LG2          (FIFO_DEPTH_LG2),
    .DATA_WIDTH         (4)
  )   AR2R_RLEN_FIFO  (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .full_o                 (ar2r_rlen_full),
    .wren_i                 (ar2r_rlen_wren),
    .wdata_i                (ar2fifo_rlen_wdata),

    .empty_o                (ar2r_rlen_empty),
    .rden_i                 (ar2r_rlen_rden),
    .rdata_o                (ar2r_rlen_rdata)
  );

  ////// R channel  //////
  assign ar2rfifo_empty = ar2r_raddr_empty | ar2r_rlen_empty | ar2r_rid_empty;

  assign ar2r_raddr_rden  = ar2rfifo_rden;
  assign ar2r_rid_rden    = ar2rfifo_rden;
  assign ar2r_rlen_rden   = ar2rfifo_rden;

  AXI_SLAVE_R   #(
    .ADDR_WIDTH             (ADDR_WIDTH),
    .DATA_WIDTH             (DATA_WIDTH),
    .ID_WIDTH               (ID_WIDTH)
  )   R_CH_FSM  (
    .clk                    (clk),
    .rst_n                  (rst_n),

    .rready_i               (r_ch.rready),
    .rvalid_o               (r_ch.rvalid),
    .rid_o                  (r_ch.rid),
//    .rdata_o                (r_ch.rdata),
    .rresp_o                (r_ch.rresp),
    .rlast_o                (r_ch.rlast),

    .raddr_i                (ar2r_raddr_rdata),
    .rid_i                  (ar2r_rid_rdata),
    .rlen_i                 (ar2r_rlen_rdata),

    .raddr_o                (raddr),
    
    .empty_i                (ar2rfifo_empty),
    .rden_o                 (ar2rfifo_rden)
  );

  always_comb begin
    if (r_ch.rvalid) begin
      for (int i=0; i<DATA_WIDTH/8; i++) begin
        r_ch.rdata[i*8 +: 8] = read_byte(raddr + i);    // [i*8+7:i*8]
      end
    end
  end
endmodule
