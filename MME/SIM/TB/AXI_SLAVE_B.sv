`include "../../RTL/AXI_TYPEDEF.svh"

module AXI_SLAVE_B
#(
  parameter ID_WIDTH          = `AXI_ID_WIDTH
)(
  input   wire                        clk,
  input   wire                        rst_n,  // _n means active low

  // AXI_B_CH interface
  output  wire                        bvalid_o,
  output  wire  [ID_WIDTH-1:0]        bid_o,
  output  wire  [1:0]                 bresp_o,
  input   wire                        bready_i,

  // signal from W_ch to B_ch
  input   wire  [ID_WIDTH-1:0]        bid_i,

  // fifo from W_ch ctrl
  input   wire                        empty_i,
  output  wire                        rden_o
);
  localparam logic            S_B_IDLE = 0,
                              S_B_RESP = 1;

  logic   [1:0]               bstate,             bstate_n;

  logic   [ID_WIDTH-1:0]      bid,                bid_n;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      bstate              <= S_B_IDLE;

      bid                 <= {ID_WIDTH{1'b0}};
    end
    else begin
      bstate              <= bstate_n;

      bid                 <= bid_n;
    end
  end

  reg   rden;
  reg   bvalid;
  always_comb begin
    rden                = 1'b0;
    bvalid              = 1'b0;

    bid_n               = bid;
    bstate_n            = bstate;

    case (bstate)
      S_B_IDLE: begin
        if (!empty_i) begin
          rden                = 1'b1;
          bid_n               = bid_i;

          bstate_n            = S_B_RESP;
        end
      end
      S_B_RESP: begin
        bvalid              = 1'b1;
        if (bready_i) begin
          bstate_n            = S_B_IDLE;
        end
      end
    endcase
  end

  assign rden_o     = rden;
  assign bvalid_o   = bvalid;
  assign bid_o      = bid;
  assign bresp_o    = 2'd0;
endmodule


