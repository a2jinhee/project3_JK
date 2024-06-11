// Copyright (c) 2021 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

module DUAL_PORT_SRAM
#(
    parameter   AW              = 8,    // address width
    parameter   DW              = 32    // data width
)
(
    input   wire                clk,
    input   wire                wren_i,
    input   wire [AW-1:0]       waddr_i,
    input   wire [DW/8-1:0]     wbyteenable_i,
    input   wire [DW-1:0]       wdata_i,
    input   wire [AW-1:0]       raddr_i,
    output  reg  [DW-1:0]       rdata_o
);

    reg [DW-1:0]                ram[2**AW-1:0]; // ** is exponential

    always @(posedge clk) begin
        if (wren_i) begin
            for (int i=0; i<DW/8; i++) begin
                // update bytes whose enable bits are set
                if (wbyteenable_i[i]) begin
                    ram[waddr_i][8*i+:8]        <= wdata_i[8*i+:8];
                end
            end
        end
    end

    always @(posedge clk) begin
        rdata_o                 <= ram[raddr_i];
    end

    // synthesis translate_off
    function write;
    input   [AW-1:0]    addr;
    input   [DW-1:0]    data;
    begin
        ram[addr]               = data;
    end
    endfunction
    // synthesis translate_on

endmodule
