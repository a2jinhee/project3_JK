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
            ADDR_A      = 3'b001,
            ADDR_B      = 3'b010,
            LOAD        = 3'b011,
            WAIT_MM     = 3'b100,
            ADDR_C      = 3'b101,
            WRITE_C     = 3'b110;

reg [2:0] state, state_n;
reg [BUF_DW-1:0] buf_a_data, buf_b_data; 
// reg [BUF_DW-1:0] buf_a_data_n, buf_b_data_n;
reg [BUF_AW-1:0] buf_a_addr, buf_b_addr, buf_a_addr_n, buf_b_addr_n;
reg [2:0] count_a, count_b, count_a_n, count_b_n;
reg [4:0] count_c, count_c_n;
reg [1:0] burst_a, burst_b;
reg [1:0] burst_a_n, burst_b_n;

// Read matrix A from memory and store into buffer A
// Read matrix B from memory and store into buffer B
// To hide DRAM latency, read matrix A and B in parallel
// Generate single-pulse start command to MM_ENGINE
// Wait for the done signal from MM_ENGINE to be 1
// Read output matrix C from MM_ENGINE and write to memory

always_ff @(posedge clk)
    if (!rst_n) begin
        state <= IDLE;
        burst_a <= 0; burst_b <= 0;
        count_c <= 0;
        count_a <= 0; count_b <= 0;
        // buf_a_data <= 0; buf_b_data <= 0;
        buf_a_addr <= 0; buf_b_addr <= 0;
    end else begin
        state <= state_n;
        burst_a <= burst_a_n; burst_b <= burst_b_n;
        count_c <= count_c_n;
        count_a <= count_a_n; count_b <= count_b_n;
        // buf_a_data <= buf_a_data_n; buf_b_data <= buf_b_data_n;
        buf_a_addr <= buf_a_addr_n; buf_b_addr <= buf_b_addr_n;
    end
    
always_comb begin
    state_n = state;
    burst_a_n = burst_a; burst_b_n = burst_b;
    count_c_n = count_c;
    count_a_n = count_a; count_b_n = count_b;
    // buf_a_data_n = buf_a_data; buf_b_data_n = buf_b_data;
    buf_a_addr_n = buf_a_addr; buf_b_addr_n = buf_b_addr;
    
    // AXI interface AR channel
    axi_ar_if.arlen = 15; axi_ar_if.arsize = 4; axi_ar_if.arburst = 1;
    axi_ar_if.arid = 0; axi_ar_if.arvalid = 0;

    // AXI interface AW channel
    axi_aw_if.awvalid = 0; axi_aw_if.awlen = 15; axi_aw_if.awsize = 4;
    axi_aw_if.awburst = 1; axi_aw_if.awid = 0;

    // AXI interface W channel
    axi_w_if.wvalid = 0; axi_w_if.wlast = 0; axi_w_if.wid = 0; axi_w_if.wstrb = 'hf;

    // AXI interface B channel
    axi_b_if.bready = 0;

    // AXI interface R channel
    axi_r_if.rready = 0;
    done_o = 1;
    mm_start_o = 0; 
    buf_a_wbyteenable_o = 'hffff;
    buf_b_wbyteenable_o = 'hffff;

    buf_a_wren_o = 0; buf_b_wren_o = 0;

    // solve latch issues
    axi_ar_if.araddr = 0;
    buf_a_data = 0; 
    buf_b_data = 0; 

    case (state)
        IDLE: begin
            if (start_i) begin
                done_o = 0;
                mm_start_o = 0; 
                state_n = ADDR_A;
                burst_a_n = 0; burst_b_n = 0;
            end
        
        end
        ADDR_A: begin
            // AR CHANNEL
            // - output: arvalid, arid, araddr, arlen, arsize, arburst
            // - input: arready
            done_o = 0;
            mm_start_o =0; 
            axi_ar_if.arvalid = 1;
            axi_ar_if.arid = 0;
            axi_ar_if.araddr = mat_a_addr_i + (burst_a * 64);

            if (axi_ar_if.arready) begin
                axi_ar_if.arvalid = 0;
                burst_a_n = burst_a + 1;
            end
            else
                state_n = ADDR_A;

            if (!axi_ar_if.arvalid && axi_ar_if.arready && burst_a == (mat_width_i / 4 - 1))
                state_n = ADDR_B;
        end
        ADDR_B: begin
            // AR CHANNEL
            // - output: arvalid, arid, araddr, arlen, arsize, arburst
            // - input: arready
            done_o = 0;
            mm_start_o = 0; 
            axi_ar_if.arvalid = 1;
            axi_ar_if.arid = 1;
            axi_ar_if.araddr = mat_b_addr_i + (burst_b * 64);

            if (axi_ar_if.arready) begin
                axi_ar_if.arvalid = 0;
                burst_b_n = burst_b + 1;
            end
            else
                state_n = ADDR_B;

            if (!axi_ar_if.arvalid && axi_ar_if.arready && burst_b == (mat_width_i / 4 - 1))
                state_n = LOAD;
        end
        LOAD: begin
            // R CHANNEL
            // - output: rready
            // - input: rvalid, rid, rdata, rlast
            done_o = 0;
            mm_start_o = 0; 
            axi_r_if.rready = 1;

            if (axi_r_if.rready && axi_r_if.rvalid && axi_r_if.rid == 0) begin
                buf_a_data[{~count_a[1:0], 5'b0}+:32] = axi_r_if.rdata;
                count_a_n = count_a + 1;
            end

            if (count_a == 3) begin
                count_a_n = 0;
                buf_a_wren_o = 1;
                buf_a_addr_n = buf_a_addr + 1;
            end

            if (axi_r_if.rready && axi_r_if.rvalid && axi_r_if.rid == 1) begin
                buf_b_data[{~count_b[1:0], 5'b0}+:32] = axi_r_if.rdata;
                count_b_n = count_b + 1;
            end

            if (count_b == 3) begin
                count_b_n = 0;
                buf_b_wren_o = 1;
                buf_b_addr_n = buf_b_addr + 1;
            end

            if (axi_r_if.rlast) begin
                if ((buf_a_addr_n == mat_width_i) && (buf_b_addr_n == mat_width_i)) begin
                    state_n = WAIT_MM;
                    mm_start_o = 1; 
                end
            end
                
        end
        WAIT_MM: begin
            buf_a_addr_n = 0;
            buf_b_addr_n = 0;
            done_o = 0;
            if (mm_start_o)
                state_n = WAIT_MM;

            if (mm_done_i && !mm_start_o)
                state_n = ADDR_C;
            
        end
        ADDR_C: begin
            // AW CHANNEL
            // - output: awvalid, awid, awaddr, awlen, awsize, awburst
            // - input: awready
            done_o = 0;
            mm_start_o = 0; 
            axi_aw_if.awvalid = 1;
            
            axi_aw_if.awaddr = mat_c_addr_i;

            if (axi_aw_if.awready)
                axi_aw_if.awvalid = 0;
            else
                state_n = ADDR_C;

            if (!axi_aw_if.awvalid && axi_aw_if.awready)
                state_n = WRITE_C;

        end
        WRITE_C: begin
            // W CHANNEL
            // - output: wvalid, wid, wdata, wlast
            // - input: wready
            // B CHANNEL
            // - output: bready
            // - input: bvalid, bid, bresp
            done_o = 0;
            mm_start_o = 0; 
            axi_w_if.wvalid = 1;
            axi_b_if.bready = 1;

            // update counter when handshake
            if (axi_w_if.wready && axi_w_if.wvalid) begin
                axi_w_if.wdata = accum_i[count_c / 4][count_c % 4];
                count_c_n = count_c + 1;
            end

            // send last signal on last write
            if (count_c == 15) begin
                axi_w_if.wlast = 1;
                count_c_n = 0;
            end
            else begin
                axi_w_if.wlast = 0;
            end

            // B channel handshake
            if (axi_b_if.bready & axi_b_if.bvalid) begin
                done_o = 1;
                state_n = IDLE;
            end
                
        end

    endcase
end

assign buf_a_waddr_o = buf_a_addr;
assign buf_b_waddr_o = buf_b_addr;

assign buf_a_wdata_o = buf_a_data;
assign buf_b_wdata_o = buf_b_data;




endmodule