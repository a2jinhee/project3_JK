`include "../../RTL/AXI_TYPEDEF.svh"

module AXI_SLAVE_AW
#(
  parameter ADDR_WIDTH        = 16,
  parameter DATA_WIDTH        = `AXI_DATA_WIDTH,
  parameter ID_WIDTH          = `AXI_ID_WIDTH,
  parameter AWREADY_DELAY     = 1
)(
  input   wire                    clk,
  input   wire                    rst_n,  // _n means active low

  // AXI_W_CH interface
  input 	wire                      awvalid_i,
  input 	wire  [ID_WIDTH-1:0]      awid_i,
  input 	wire  [ADDR_WIDTH-1:0]    awaddr_i,
  input 	wire  [3:0]               awlen_i,
  input 	wire  [2:0]               awsize_i,
  input 	wire  [1:0]               awburst_i,
  output	wire                      awready_o,

  // signal from AW_ch to W_ch
  output  wire  [ADDR_WIDTH-1:0]    waddr_o,
  output  wire  [ID_WIDTH-1:0]      wid_o,
  output  wire  [3:0]               wlen_o,

  // fifo ctrl
  input   wire                      full_i,
  output  wire                      wren_o
);
  localparam logic [1:0]      S_AW_IDLE = 0,
                              S_AW_READY = 1,
                              S_AW_PUSH = 2;

  logic   [1:0]               awstate,            awstate_n;
  logic   [7:0]               wcnt,               wcnt_n;

  logic   [ADDR_WIDTH-1:0]    waddr,              waddr_n;
  logic   [ID_WIDTH-1:0]      wid,                wid_n;
  logic   [3:0]               wlen,               wlen_n;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      awstate             <= S_AW_IDLE;

      wcnt                <= 8'd0;
      waddr               <= {ADDR_WIDTH{1'b0}};
      wid                 <= {ID_WIDTH{1'b0}};
      wlen                <= 4'd0;
    end
    else begin
      awstate             <= awstate_n;

      wcnt                <= wcnt_n;
      waddr               <= waddr_n;
      wid                 <= wid_n;
      wlen                <= wlen_n;
    end
  end

  reg   awready;
  reg   wren;
  always_comb begin
    awstate_n               = awstate;

    wcnt_n                  = wcnt;
    waddr_n                 = waddr;
    wid_n                   = wid;
    wlen_n                  = wlen;

    wren                    = 1'b0;
    awready                 = 1'b0;

    case (awstate)
      S_AW_IDLE: begin
        if (awvalid_i) begin
          if (AWREADY_DELAY == 0) begin
            waddr_n                 = awaddr_i;
            wid_n                   = awid_i;
            wlen_n                  = awlen_i;
            awready                 = full_i ? 1'b0 : 1'b1;
            awstate_n               = S_AW_PUSH;
          end
          else begin
            wcnt_n                  = AWREADY_DELAY-1;
            awstate_n               = S_AW_READY;
          end
        end
      end
      S_AW_READY: begin
        if (wcnt==0) begin
          waddr_n                 = awaddr_i;
          wid_n                   = awid_i;
          wlen_n                  = awlen_i;
          awready                 = 1'b1;
          awstate_n               = S_AW_PUSH;
        end
        else begin
          wcnt_n                  = wcnt - 8'd1;
        end
      end
      S_AW_PUSH: begin
        if (!full_i) begin
          wren                    = 1'b1;

          awstate_n               = S_AW_IDLE;
        end
      end
    endcase
  end

  assign wren_o     = wren;
  assign awready_o  = awready;
  assign waddr_o    = waddr;
  assign wid_o      = wid;
  assign wlen_o     = wlen;
endmodule

