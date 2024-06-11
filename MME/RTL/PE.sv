// Copyright (c) 2021 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

module PE
#(
    parameter DW                = 32    // data width
 )
(
    input   wire                clk,
    input   wire                rst_n,
    input   wire                clr_i,
    input   wire                hold_i,
    input   signed [DW-1:0]     a_i,
    input   signed [DW-1:0]     b_i,
    output  signed [DW-1:0]     a_o,
    output  signed [DW-1:0]     b_o,
    output  signed [2*DW:0]     accum_o
);
    reg signed [DW-1:0]         a_reg;
    reg signed [DW-1:0]         b_reg;
    reg signed [2*DW:0]         accum;

    always_ff @(posedge clk)
    begin
        if (!rst_n | clr_i) begin
            a_reg                   <= 'd0;
            b_reg                   <= 'd0;
            accum                   <= 'd0;
        end
        else if (!hold_i) begin
            a_reg                   <= a_i;
            b_reg                   <= b_i;
            accum                   <= accum + (a_i * b_i);
        end
    end

    assign  a_o                 = a_reg;
    assign  b_o                 = b_reg;
    assign  accum_o             = accum;


endmodule
