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

    // FSM states
localparam  IDLE        = 3'b000,
            ADDR_A_B    = 3'b001,
            LOAD        = 3'b010,
            WAIT_MM     = 3'b011,
            WRITE_C     = 3'b100;

reg [2:0] state, state_n;
reg [BUF_DW-1:0] buf_a_data, buf_b_data;
reg [BUF_AW-1:0] buf_a_addr, buf_b_addr;
reg [4:0] count_c; 
reg [3:0] burst_count; 

// Simplify and merge states for reading A and B addresses
always_comb begin 
    state_n = state;
    done_o = 0;

    // Default AXI settings
    axi_ar_if.arvalid = 0;
    axi_aw_if.awvalid = 0;
    axi_w_if.wvalid = 0;
    axi_b_if.bready = 0;
    axi_r_if.rready = 0;

    case (state)
        IDLE: begin
            if (start_i) begin
                done_o = 0;
                state_n = ADDR_A_B;
            end
        end
        ADDR_A_B: begin
            done_o = 0;
            // Setup for burst read from A and B addresses
            axi_ar_if.arvalid = 1;
            axi_ar_if.arid = 0; 
            axi_ar_if.araddr = (burst_count < (mat_width_i / 4)) ? 
                                (mat_a_addr_i + (burst_count * 64)) : 
                                (mat_b_addr_i + ((burst_count - (mat_width_i / 4)) * 64)); 

            if (axi_ar_if.arready) begin
                burst_count <= burst_count + 1;
                axi_ar_if.arvalid = 0; 
            end

            if (burst_count == (mat_width_i / 2)) begin
                state_n = LOAD;
                burst_count <= 0;
            end
        end
        LOAD: begin
            done_o = 0;
            axi_r_if.rready = 1;

            if (axi_r_if.rready && axi_r_if.rvalid) begin
                if (axi_r_if.rid == 0) begin
                    buf_a_wbyteenable_o <= 'hffff;
                    buf_a_data <= (buf_a_data << 32) | axi_r_if.rdata;
                    buf_a_wren_o <= 1;
                    buf_a_addr <= buf_a_addr + 1;
                end else begin
                    buf_b_wbyteenable_o <= 'hffff;
                    buf_b_data <= (buf_b_data << 32) | axi_r_if.rdata;
                    buf_b_wren_o <= 1;
                    buf_b_addr <= buf_b_addr + 1;
                end

                if ((buf_a_addr == mat_width_i) && (buf_b_addr == mat_width_i)) begin
                    mm_start_o <= 1;
                    state_n = WAIT_MM;
                end
            end
        end
        WAIT_MM: begin
            done_o = 0;
            if (mm_done_i) begin
                state_n = WRITE_C;
            end
        end
        WRITE_C: begin
            done_o = 0;
            axi_aw_if.awvalid = 1;
            axi_aw_if.awaddr = mat_c_addr_i;

            if (axi_aw_if.awready) begin
                axi_aw_if.awvalid = 0;
                axi_w_if.wvalid = 1;
                axi_b_if.bready = 1;

                if (axi_w_if.wready) begin
                    axi_w_if.wdata = accum_i[count_c / 4][count_c % 4];
                    count_c <= count_c + 1;
                    axi_w_if.wlast = (count_c == 15);

                    if (axi_b_if.bready && axi_b_if.bvalid) begin
                        done_o = 1;
                        state_n = IDLE;
                    end
                end
            end
        end
    endcase
end

// Counters and addresses
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        buf_a_addr <= 0; buf_b_addr <= 0;
        buf_a_data <= 0; buf_b_data <= 0;
        count_c <= 0;
        burst_count <= 0;
        mm_start_o <= 0;
    end else begin
        buf_a_wren_o <= 0; buf_b_wren_o <= 0;

        case (state)
            ADDR_A_B: begin
                if (axi_ar_if.arready) burst_count <= burst_count + 1;
            end
            LOAD: begin
                // handled in always_comb block
            end
            WRITE_C: begin
                if (axi_w_if.wready && axi_w_if.wvalid) count_c <= count_c + 1;
            end
        endcase
    end
end

assign buf_a_waddr_o = buf_a_addr;
assign buf_a_wdata_o = buf_a_data;

assign buf_b_waddr_o = buf_b_addr;
assign buf_b_wdata_o = buf_b_data;



    
endmodule
