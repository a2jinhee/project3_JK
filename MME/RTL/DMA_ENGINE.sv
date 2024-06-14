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

    // Read matrix A from memory and store into buffer A
    // Read matrix B from memory and store into buffer B
    // To hide DRAM latency, read matrix A and B in parallel


    // Generate single-pulse start command to MM_ENGINE
    // Wait for the done signal from MM_ENGINE to be 1

    // Read output matrix C from MM_ENGINE and write to memory


    localparam  IDLE        = 3'b000,
                READ_A      = 3'b001,
                READ_B      = 3'b010,
                WAIT_MM     = 3'b011,
                WRITE_C     = 3'b100;

    reg [2:0] state, state_n;
    reg [7:0] a_read_count, b_read_count, c_write_count;
    reg [BUF_AW-1:0] buf_a_addr, buf_b_addr, buf_c_addr;
    reg [BUF_DW-1:0] buf_a_data, buf_b_data;
    reg [1:0] a_burst_count, b_burst_count, c_burst_count;
    
    always_ff @(posedge clk)
        if (!rst_n)
            state <= IDLE;
        else
            state <= state_n;

    always_comb begin 
        state_n = state;
        case (state)
            IDLE: begin
                if (start_i)
                    state_n = READ_A;
            end
            READ_A: begin
                if (a_read_count== SA_WIDTH*mat_width_i - 1)
                    // handshake with axi_if; 
                    // pass mat_b_addr_i + offset to axi_if to get mem_b data
                    state_n = READ_B;
                else
                    state_n = READ_A;
            end
            READ_B: begin
                if (b_read_count == SA_WIDTH*mat_width_i - 1)
                    state_n = WAIT_MM;
                else
                    state_n = READ_B;
            end
            WAIT_MM: begin
                if (mm_done_i)
                    state_n = WRITE_C;
                else
                    state_n = WAIT_MM;
            end
            WRITE_C: begin
                if (c_write_count == 3)
                    state_n = IDLE;
            end
        endcase
    end

    // Counters and addresses
    always @(posedge clk) begin
        if (!rst_n) begin
            a_read_count <= 0;
            b_read_count <= 0;
            c_write_count <= 0;

            axi_ar_if.araddr <= 0;
            axi_ar_if.arvalid <= 0;
            axi_aw_if.awaddr <= 0;
            axi_aw_if.awvalid <= 0;

        end else begin
            case (state)
                READ_A: begin
                    // axi command to read data from mat_a_addr_i + offset (burst for 128 bits = 16 bytes)
                    axi_ar_if.arlen <= 16;
                    axi_ar_if.arsize <= 4;
                    axi_ar_if.arburst <= 1;

                    axi_ar_if.araddr <= mat_a_addr_i + a_read_count * 16 * (DW/4); // byte address

                    if (axi_r_if.rlast)
                        a_read_count <= a_read_count + 1;
                        axi_ar_if.arvalid <= 1;

                end
                READ_B: begin
                    $display("READ_B!!!");
                    // axi command to read data from mat_a_addr_i + offset (burst for 128 bits = 16 bytes)
                    axi_ar_if.arlen <= 16;
                    axi_ar_if.arsize <= 4;
                    axi_ar_if.arburst <= 1;

                    axi_ar_if.araddr <= mat_b_addr_i + b_read_count * 16 * (DW/4); // byte address

                    if (axi_r_if.rlast)
                        b_read_count <= b_read_count + 1;
                        axi_ar_if.arvalid <= 1;

                end
                WRITE_C: begin
                    axi_aw_if.awlen <= 16;
                    axi_aw_if.awsize <= 4;
                    axi_aw_if.awburst <= 1;

                    axi_aw_if.awaddr <= mat_c_addr_i + c_write_count * 16 * (DW/4); // byte address

                    if (axi_w_if.wlast)
                        c_write_count <= c_write_count + 1;
                end
            endcase
        end
        $display("state: %d, state_n: %d\n", state, state_n);
        $display("a_read_count: %d, b_read_count: %d, c_write_count: %d\n", a_read_count, b_read_count, c_write_count);
        $display("axi_ar_if.araddr: %d\n", axi_ar_if.araddr);
        
        // $display("buf_a_addr: %d, buf_b_addr: %d, buf_c_addr: %d\n", buf_a_addr, buf_b_addr, buf_c_addr);
        // $display("mat_a_addr_i: %d, mat_b_addr_i: %d, mat_c_addr_i: %d\n", mat_a_addr_i, mat_b_addr_i, mat_c_addr_i);
    end

    // FSM output logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_a_wren_o <= 0;
            buf_b_wren_o <= 0;
            mm_start_o <= 0;
            done_o <= 0;

            buf_a_data <= 0;
            buf_b_data <= 0;

            buf_a_addr <= 0; 
            buf_b_addr <= 0;

            axi_r_if.rready <= 0;

            axi_aw_if.awaddr <= 0;
            axi_w_if.wdata <= 0;
            axi_w_if.wvalid <= 0;
            axi_aw_if.awvalid <= 0;

            a_burst_count <= 0;
            b_burst_count <= 0;
            c_burst_count <= 0;


        end else begin
            buf_a_wren_o <= 0;
            buf_b_wren_o <= 0;
            mm_start_o <= 0;
            done_o <= 0;
            case (state)
                READ_A: begin
                    buf_a_wren_o <= 0; 
                    axi_r_if.rready <= 1;

                    if (axi_r_if.rvalid)
                        buf_a_addr <= buf_a_addr + a_burst_count * (DW / 8);
                        buf_a_data <= axi_r_if.rdata;  // Assuming axi_r_if provides the read data
                        a_burst_count <= a_burst_count + 1;

                        if (a_burst_count == 3)
                            a_burst_count <= 0;
                            buf_a_wren_o <= 1;
                    
                end
                READ_B: begin
                    buf_b_wren_o <= 0;
                    axi_r_if.rready <= 1;

                    if (axi_r_if.rvalid)
                        buf_b_addr <= buf_b_addr + b_burst_count * (DW / 8);
                        buf_b_data <= axi_r_if.rdata;  // Assuming axi_r_if provides the read data
                        b_burst_count <= b_burst_count + 1;

                        if (b_burst_count == 3)
                            b_burst_count <= 0;
                            buf_b_wren_o <= 1;

                end
                WAIT_MM: begin
                    mm_start_o <= 1;
                end
                WRITE_C: begin

                    if (axi_w_if.wready)
                        axi_w_if.wdata <= accum_i[c_write_count][c_burst_count];  // Assuming accum_i provides the computed data
                        c_burst_count <= c_burst_count + 1;
                        axi_w_if.wvalid <= 1;

                        if (c_burst_count == 3)
                            c_burst_count <= 0;
                            axi_w_if.wlast <= 1;

                    if (c_write_count == 3)
                        done_o <= 1;
                end
            endcase
        end
        $display("buf_a_data: %d, buf_b_data: %d\n", buf_a_data, buf_b_data);
        $display("axi_r_if.rdata: \n", axi_r_if.rdata);
    end

    assign buf_a_waddr_o = a_read_count;
    assign buf_b_waddr_o = b_read_count;

    assign buf_a_wdata_o = buf_a_data;
    assign buf_b_wdata_o = buf_b_data;

endmodule
