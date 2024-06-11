`include "../../RTL/AXI_TYPEDEF.svh"

module AXI_SLAVE_AR
#(
  parameter ADDR_WIDTH        = 16,
  parameter DATA_WIDTH        = `AXI_DATA_WIDTH,
  parameter ID_WIDTH          = `AXI_ID_WIDTH,
  parameter ARREADY_DELAY     = 1,
  parameter AR2R_DELAY        = 50
)(
  input   wire                    clk,
  input   wire                    rst_n,  // _n means active low

  // AXI_AR_CH interface
  input   wire                    arvalid_i,
  input   wire  [ID_WIDTH-1:0]    arid_i,
  input   wire  [ADDR_WIDTH-1:0]  araddr_i,
  input   wire  [3:0]             arlen_i,
  input   wire  [2:0]             arsize_i,
  input   wire  [1:0]             arburst_i,
  output  wire                    arready_o,

  // signal from AR_ch to R_ch
  output  wire  [ADDR_WIDTH-1:0]  raddr_o,
  output  wire  [ID_WIDTH-1:0]    rid_o,
  output  wire  [3:0]             rlen_o,

  // fifo ctrl
  input   wire                    full_i,
  output  wire                    wren_o
);
  localparam logic [1:0]      S_AR_IDLE = 0,
                              S_AR_READY = 1,
                              S_AR_DELAY = 2,
                              S_AR_PUSH = 3;

  logic   [1:0]               arstate,            arstate_n;
  logic   [7:0]               rcnt,               rcnt_n;

  logic   [ADDR_WIDTH-1:0]    raddr,              raddr_n;
  logic   [ID_WIDTH-1:0]      rid,                rid_n;
  logic   [3:0]               rlen,               rlen_n;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      arstate             <= S_AR_IDLE;

      rcnt                <= 8'd0;
      raddr               <= {ADDR_WIDTH{1'b0}};
      rid                 <= {ID_WIDTH{1'b0}};
      rlen                <= 4'd0;
    end
    else begin
      arstate             <= arstate_n;

      rcnt                <= rcnt_n;
      raddr               <= raddr_n;
      rid                 <= rid_n;
      rlen                <= rlen_n;
    end
  end

  reg   arready;
  reg   wren;
  always_comb begin
    arstate_n               = arstate;

    rcnt_n                  = rcnt;
    raddr_n                 = raddr;
    rid_n                   = rid;
    rlen_n                  = rlen;

    wren                    = 1'b0;
    arready                 = 1'b0;

    case (arstate)
      S_AR_IDLE: begin
        if (arvalid_i) begin
          if (ARREADY_DELAY == 0) begin
            raddr_n                 = araddr_i;
            rid_n                   = arid_i;
            rlen_n                  = arlen_i;
            arready                 = full_i ? 1'b0 : 1'b1;

            rcnt_n                  = AR2R_DELAY - 1;
            arstate_n               = S_AR_DELAY;
          end
          else begin
            rcnt_n                  = ARREADY_DELAY-1;
            arstate_n               = S_AR_READY;
          end
        end
      end
      S_AR_READY: begin
        if (rcnt==0) begin
          raddr_n                 = araddr_i;
          rid_n                   = arid_i;
          rlen_n                  = arlen_i;
          arready                 = full_i ? 1'b0 : 1'b1;

          rcnt_n                  = AR2R_DELAY - 1;
          arstate_n               = S_AR_DELAY;
        end
        else begin
          rcnt_n                  = rcnt - 8'd1;
        end
      end
      S_AR_DELAY: begin
        if (rcnt==0) begin
          arstate_n               = S_AR_PUSH;
        end
        else begin
          rcnt_n                  = rcnt - 8'd1;
        end
      end
      S_AR_PUSH: begin
        if (!full_i) begin
          wren                      = 1'b1;

          arstate_n                 = S_AR_IDLE;
        end
      end
    endcase
  end

  assign wren_o     = wren;
  assign arready_o  = arready;
  assign raddr_o    = raddr;
  assign rid_o      = rid;
  assign rlen_o     = rlen;
endmodule
