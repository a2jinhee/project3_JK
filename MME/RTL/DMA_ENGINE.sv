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

    //// CHANGES ////

    // Internal states for FSM
    typedef enum logic [1:0] {
        IDLE,
        READ_A,
        READ_B,
        WAIT_MM,
        WRITE_C
    } state_t;

    state_t state, next_state;

    // Counters and addresses
    reg [7:0] a_read_count, b_read_count, c_write_count;
    reg [31:0] a_addr, b_addr, c_addr;

    // Buffer address
    reg [BUF_AW-1:0] buf_a_addr, buf_b_addr;

    // Internal signals for data
    reg [BUF_DW-1:0] buf_a_data, buf_b_data;

    // State transition
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    // Next state logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (start_i)
                    next_state = READ_A;
            end
            READ_A: begin
                if (a_read_count == mat_width_i * SA_WIDTH - 1)
                    next_state = READ_B;
            end
            READ_B: begin
                if (b_read_count == mat_width_i * SA_WIDTH - 1)
                    next_state = WAIT_MM;
            end
            WAIT_MM: begin
                if (mm_done_i)
                    next_state = WRITE_C;
            end
            WRITE_C: begin
                if (c_write_count == mat_width_i * SA_WIDTH - 1)
                    next_state = IDLE;
            end
        endcase
    end

    // Counters and addresses
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_read_count <= 0;
            b_read_count <= 0;
            c_write_count <= 0;
            a_addr <= 0;
            b_addr <= 0;
            c_addr <= 0;
        end else begin
            case (state)
                READ_A: begin
                    a_read_count <= a_read_count + 1;
                    a_addr <= mat_a_addr_i + a_read_count * DW / 8;
                end
                READ_B: begin
                    b_read_count <= b_read_count + 1;
                    b_addr <= mat_b_addr_i + b_read_count * DW / 8;
                end
                WRITE_C: begin
                    c_write_count <= c_write_count + 1;
                    c_addr <= mat_c_addr_i + c_write_count * DW / 8;
                end
                default: begin
                    a_read_count <= 0;
                    b_read_count <= 0;
                    c_write_count <= 0;
                    a_addr <= mat_a_addr_i;
                    b_addr <= mat_b_addr_i;
                    c_addr <= mat_c_addr_i;
                end
            endcase
        end
    end

    // AXI read and write operations (simplified for brevity)
    // Assign buffer addresses and data
    assign buf_a_waddr_o = buf_a_addr;
    assign buf_b_waddr_o = buf_b_addr;
    assign buf_a_wdata_o = buf_a_data;
    assign buf_b_wdata_o = buf_b_data;

    // FSM output logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_a_wren_o <= 0;
            buf_b_wren_o <= 0;
            buf_a_wbyteenable_o <= 0;
            buf_b_wbyteenable_o <= 0;
            mm_start_o <= 0;
            done_o <= 0;
        end else begin
            buf_a_wren_o <= 0;
            buf_b_wren_o <= 0;
            mm_start_o <= 0;
            done_o <= 0;
            case (state)
                READ_A: begin
                    buf_a_wren_o <= 1;
                    buf_a_wbyteenable_o <= {BUF_DW/8{1'b1}};
                    buf_a_data <= axi_r_if.rdata;  // Assuming axi_r_if provides the read data
                end
                READ_B: begin
                    buf_b_wren_o <= 1;
                    buf_b_wbyteenable_o <= {BUF_DW/8{1'b1}};
                    buf_b_data <= axi_r_if.rdata;  // Assuming axi_r_if provides the read data
                end
                WAIT_MM: begin
                    mm_start_o <= 1;
                end
                WRITE_C: begin
                    axi_aw_if.awaddr <= c_addr;
                    axi_w_if.wdata <= accum_i;  // Assuming accum_i provides the computed data
                    axi_w_if.wvalid <= 1;
                    if (c_write_count == mat_width_i * SA_WIDTH - 1)
                        done_o <= 1;
                end
            endcase
        end
    end
    

endmodule
