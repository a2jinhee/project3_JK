// Copyright (c) 2021 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

module MM_ENGINE
#(
    parameter DW                = 32,   // data width
    parameter SA_WIDTH          = 4,    // systolic array width in PE count
    parameter BUF_AW            = 6,    // buffer address width
    parameter BUF_DW            = DW * SA_WIDTH // buffer data width
 )
(
    input   wire                clk,
    input   wire                rst_n,  // _n means active low

    input   wire    [7:0]       mat_width_i,
    input   wire                start_i,
    output  wire                done_o,

    output  wire [BUF_AW-1:0]   buf_a_raddr_o,
    input   wire [BUF_DW-1:0]   buf_a_rdata_i,
    output  wire [BUF_AW-1:0]   buf_b_raddr_o,
    input   wire [BUF_DW-1:0]   buf_b_rdata_i,

    output  signed [2*DW:0]     accum_o[SA_WIDTH][SA_WIDTH]
);

    wire    signed [DW-1:0]     a[SA_WIDTH];
    wire    signed [DW-1:0]     b[SA_WIDTH];

    wire                        buf_a_wren;
    wire    [BUF_AW-1:0]        buf_a_waddr,    buf_a_raddr;
    wire    [BUF_DW/8-1:0]      buf_a_wbyteenable, buf_b_wbyteenable;
    wire    [BUF_DW-1:0]        buf_a_wdata,    buf_a_rdata;
    wire                        buf_b_wren;
    wire    [BUF_AW-1:0]        buf_b_waddr,    buf_b_raddr;
    wire    [BUF_DW-1:0]        buf_b_wdata,    buf_b_rdata;

    reg                         dp_start,       sa_start;
    wire                        dp_done,        sa_done;

    // Start signals
    assign  dp_start            = start_i;
    // add one-cycle delay to SA start to reflect data preparation time
    always @(posedge clk)
        if (!rst_n) begin
            sa_start                    <= 1'b0;
        end
        else begin
            sa_start                    <= start_i;
        end

    // complete when both modules are done
    assign  done_o              = dp_done & sa_done;

    DATA_PROVIDER
    #(
        .DW                     (DW),
        .SA_WIDTH               (SA_WIDTH),
        .BUF_AW                 (BUF_AW),
        .BUF_DW                 (BUF_DW)
    )
    u_provider_a
    (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .mat_width_i            (mat_width_i),
        .start_i                (dp_start),
        .done_o                 (dp_done),
        .sram_addr_o            (buf_a_raddr_o),
        .sram_data_i            (buf_a_rdata_i),
        .a_o                    (a)
    );

    DATA_PROVIDER
    #(
        .DW                     (DW),
        .SA_WIDTH               (SA_WIDTH),
        .BUF_AW                 (BUF_AW),
        .BUF_DW                 (BUF_DW)
    )
    u_provider_b
    (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .mat_width_i            (mat_width_i),
        .start_i                (dp_start),
        .done_o                 (/* FLOATING */),
        .sram_addr_o            (buf_b_raddr_o),
        .sram_data_i            (buf_b_rdata_i),
        .a_o                    (b)
    );

    SYSTOLIC_ARRAY
    #(
        .DW                     (DW),
        .SA_WIDTH               (SA_WIDTH)
    ) 
	u_sa
    (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .mat_width_i            (mat_width_i),
        .start_i                (sa_start),
        .done_o                 (sa_done),
        .a_i                    (a),
        .b_i                    (b),
        .accum_o                (accum_o)
    );

endmodule
