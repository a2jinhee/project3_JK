// Copyright (c) 2021 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

module DATA_PROVIDER
#(
    parameter DW                = 32,   // data width
    parameter SA_WIDTH          = 4,    // systolic array width in PE count
    parameter BUF_AW            = 6,    // buffer address width
    parameter BUF_DW            = DW * SA_WIDTH // buffer data width
 )
(
    input   wire                clk,
    input   wire                rst_n,
    input   wire    [7:0]       mat_width_i,
    input   wire                start_i,
    output  wire                done_o,
    // SRAM read interface
    output  wire    [BUF_AW-1:0] sram_addr_o,
    input   wire    [BUF_DW-1:0] sram_data_i,
    output  signed  [DW-1:0]    a_o[SA_WIDTH]
);

    //-------------------------------------------------------------
    // Waveform
    //--------------------------------------------------------------
    // clk      : __--__--__--__--__--__--__--__--__--__--__--__--__
    // start    : ___----___________________________________________
    // addr     :   0   | 1 | 2 | 3 | 4 | 5 | 6 | 7 
    // data     :     D0    |D1 |D2 |D3 |D4 |D5 |D6 | D7
    // a_o[0]   :     D0    |D1 |D2 |D3 |D4 |D5 |D6 | D7   
    // a_o[1]   :       D0      |D1 |D2 |D3 |D4 |D5 |D6 | D7   
    // a_o[2]   :         D0        |D1 |D2 |D3 |D4 |D5 |D6 | D7   
    // a_o[3]   :           D0          |D1 |D2 |D3 |D4 |D5 |D6 | D7   

    //----------------------------------------------------------
    // Control logic
    //----------------------------------------------------------
    enum reg {S_IDLE, S_BUSY}   state,  state_n;
    // +1 bit for additional cycles
    reg     [BUF_AW:0]          addr,   addr_n;
    reg                         complete;

    always_comb begin
        state_n                 = state;
        addr_n                  = addr;

        complete                = (addr==mat_width_i+SA_WIDTH-'d1);

        if (state==S_IDLE) begin
            if (start_i) begin
                state_n                 = S_BUSY;
                addr_n                  = 'd1;
            end
        end
        else begin  // S_BUSY
            if (complete) begin
                state_n                 = S_IDLE;
                addr_n                  = 'd0;
            end
            else begin
                addr_n                  = addr+'d1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state                   <= S_IDLE;
            addr                    <= 'd0;
        end
        else begin
            state                   <= state_n;
            addr                    <= addr_n;
        end
    end

    // shift registers to provide aligned data
    reg     [BUF_DW-1:0]        sram_data_reg[SA_WIDTH-1];
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i=0; i<(SA_WIDTH-1); i++) begin
                sram_data_reg[i]        <= 'd0;
            end
        end
        else begin
            sram_data_reg[0]        <= sram_data_i;
            for (int i=1; i<(SA_WIDTH-1); i++) begin
                sram_data_reg[i]        <= sram_data_reg[i-1];
            end
        end
    end

    reg     signed  [DW-1:0]    a[SA_WIDTH];

    always_comb begin
        a[0]                = signed'(sram_data_i[DW*(SA_WIDTH-1)+:DW]);
        for (int i=1; i<SA_WIDTH; i++) begin
            a[i]                = signed'(sram_data_reg[i-1][DW*(SA_WIDTH-1-i)+:DW]);
        end
    end

    // output assignments
    assign  sram_addr_o             = addr[BUF_AW-1:0];
    assign  done_o                  = (state==S_IDLE);
    assign  a_o                     = a;

endmodule
