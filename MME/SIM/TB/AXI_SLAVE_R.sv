`include "../../RTL/AXI_TYPEDEF.svh"

module AXI_SLAVE_R
#(
  parameter ADDR_WIDTH        = 16,
  parameter DATA_WIDTH        = `AXI_DATA_WIDTH,
  parameter ID_WIDTH          = `AXI_ID_WIDTH
)(
  input   wire                    clk,
  input   wire                    rst_n,  // _n means active low

  // AXI_R_CH interface
  input  wire                     rready_i,
  output wire                     rvalid_o,
  output wire   [ID_WIDTH-1:0]    rid_o,
//  output wire   [DATA_WIDTH-1:0]  rdata_o,
  output wire   [1:0]             rresp_o,
  output wire                     rlast_o,

  // signal from AR_ch to R_ch
  input   wire  [ADDR_WIDTH-1:0]  raddr_i,
  input   wire  [ID_WIDTH-1:0]    rid_i,
  input   wire  [3:0]             rlen_i,

  // signal to outside
  output  wire  [ADDR_WIDTH-1:0]  raddr_o,

  // fifo control
  input   wire                    empty_i,
  output  wire                    rden_o
);
  localparam logic            S_R_IDLE = 0,
                              S_R_BURST = 1;

  logic   [1:0]               rstate,             rstate_n;

  logic   [ADDR_WIDTH-1:0]    raddr,              raddr_n;
  logic   [ID_WIDTH-1:0]      rid,                rid_n;
  logic   [3:0]               rlen,               rlen_n;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rstate              <= S_R_IDLE;

      raddr               <= {ADDR_WIDTH{1'b0}};
      rid                 <= {ID_WIDTH{1'b0}};
      rlen                <= 4'd0;
    end
    else begin
      rstate              <= rstate_n;

      raddr               <= raddr_n;
      rid                 <= rid_n;
      rlen                <= rlen_n;
    end
  end

  reg   rden;
  reg   rvalid, rlast;
  always_comb begin
    rden                  = 1'b0;

    raddr_n               = raddr;
    rid_n                 = rid;
    rlen_n                = rlen;

    rvalid                = 1'b0;
    rlast                 = 1'b0;

    rstate_n              = rstate;

    case (rstate)
      S_R_IDLE: begin
        if (!empty_i) begin
          rden                = 1'b1;
          raddr_n             = raddr_i;
          rid_n               = rid_i;
          rlen_n              = rlen_i;

          rstate_n            = S_R_BURST;
        end
      end
      S_R_BURST: begin
        rvalid            = 1'b1;
        rlast             = (rlen == 4'd0);
        if (rready_i) begin
          raddr_n             = raddr + (DATA_WIDTH/8);
          if (rlen == 4'd0) begin
            rstate_n            = S_R_IDLE;
          end
          else begin
            rlen_n              = rlen - 4'd1;
          end
        end
      end
    endcase
  end

  assign  rden_o            = rden;
  assign  rvalid_o          = rvalid;
  assign  rlast_o           = rlast;
  assign  rid_o             = rid;
  assign  raddr_o           = raddr;
  assign  rresp_o           = 2'd0;
endmodule

