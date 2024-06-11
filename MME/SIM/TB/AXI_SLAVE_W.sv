`include "../../RTL/AXI_TYPEDEF.svh"

module AXI_SLAVE_W
#(
  parameter ADDR_WIDTH        = 16,
  parameter DATA_WIDTH        = `AXI_DATA_WIDTH,
  parameter ID_WIDTH          = `AXI_ID_WIDTH
)(
  input   wire                      clk,
  input   wire                      rst_n,  // _n means active low

  // AXI_W_CH interface
  input  wire                       wvalid_i,
  input  wire   [ID_WIDTH-1:0]      wid_i,
//  input  wire   [DATA_WIDTH-1:0]    wdata_i;
  input  wire   [DATA_WIDTH/8-1:0]  wstrb_i,
  input  wire                       wlast_i,
  output wire                       wready_o,

  // signal from AW_ch to W_ch
  input   wire  [ADDR_WIDTH-1:0]    awaddr_i,
  input   wire  [ID_WIDTH-1:0]      awid_i,
  input   wire  [3:0]               awlen_i,

  // signal to outside
  output  wire  [ADDR_WIDTH-1:0]    waddr_o,

  // fifo from AW_ch ctrl
  input   wire                      empty_i,
  output  wire                      rden_o,

  // fifo to B_ch ctrl
  input   wire                      full_i,
  output  wire                      wren_o
);
  localparam logic            S_W_IDLE = 0,
                              S_W_BURST = 1;

  logic   [1:0]               wstate,             wstate_n;

  logic   [ADDR_WIDTH-1:0]    waddr,              waddr_n;
  logic   [ID_WIDTH-1:0]      wid,                wid_n;
  logic   [3:0]               awlen,              awlen_n;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wstate              <= S_W_IDLE;

      waddr               <= {ADDR_WIDTH{1'b0}};
      wid                 <= {ID_WIDTH{1'b0}};
      awlen               <= 4'd0;
    end
    else begin
      wstate              <= wstate_n;

      waddr               <= waddr_n;
      wid                 <= wid_n;
      awlen               <= awlen_n;
    end
  end

  reg   rden, wren;
  reg   wready;
  always_comb begin
    rden                  = 1'b0;
    wren                  = 1'b0;

    waddr_n               = waddr;
    wid_n                 = wid;
    awlen_n               = awlen;

    wready                = 1'b0;

    wstate_n              = wstate;

    case (wstate)
      S_W_IDLE: begin
        if (!empty_i) begin
          rden                = 1'b1;
          waddr_n             = awaddr_i;
          wid_n               = awid_i;
          awlen_n             = awlen_i;

          wstate_n            = S_W_BURST;
        end
      end
      S_W_BURST: begin
        wready            = 1'b1;
        if (wvalid_i) begin
          waddr_n             = waddr + (DATA_WIDTH/8);
          if (awlen == 4'd0) begin
            if (!full_i) begin
              wren                = 1'b1;

              wstate_n            = S_W_IDLE;
            end
          end
          else begin
            awlen_n             = awlen - 4'd1;
          end
        end
      end
    endcase
  end

  assign  rden_o            = rden;
  assign  wren_o            = wren;
  assign  wready_o          = wready;
  assign  waddr_o           = waddr;
endmodule
