// Copyright (c) 2021 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

`define 	TIMEOUT_DELAY 	99999999

module DATA_PROVIDER_TB ();

    //----------------------------------------------------------
    // clock and reset generation
    //----------------------------------------------------------
    reg                     clk;
    reg                     rst_n;

    // clock generation
    initial begin
        clk                     = 1'b0;

        forever #10 clk         = !clk;
    end

    // reset generation
    initial begin
        rst_n                   = 1'b0;     // active at time 0

        repeat (3) @(posedge clk);          // after 3 cycles,
        rst_n                   = 1'b1;     // release the reset
    end

	// timeout
	initial begin
		#`TIMEOUT_DELAY $display("Timeout!");
		$finish;
	end

    // enable waveform dump
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, u_DUT);
    end
    initial begin
        $fsdbDumpfile("dump.fsdb");
        $fsdbDumpvars(0, u_DUT);
    end

    localparam                  AW      = 6;
    localparam                  DW      = 32;
    localparam                  SIZE    = 4;
    localparam  reg [7:0]       LENGTH  = 8'd8;

    reg                         start;
    wire                        done;
    wire    [DW-1:0]            a_w[SIZE];
    reg                         sram_wren;
    reg     [AW-1:0]            sram_waddr,     sram_raddr;
    reg     [DW*SIZE-1:0]       sram_wdata,     sram_rdata;

    DATA_PROVIDER
    #(
        .DW                     (DW),
        .SA_WIDTH               (SIZE),
        .BUF_AW                 (AW)
    )
    u_DUT
    (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .mat_width_i            (LENGTH),
        .start_i                (start),
        .done_o                 (done),
        .sram_addr_o            (sram_raddr),
        .sram_data_i            (sram_rdata),
        .a_o                    (a_w)
    );

    DUAL_PORT_SRAM
    #(
        .AW                     (AW),
        .DW                     (DW*SIZE)
     )
    u_mem
    (
        .clk                    (clk),
        .wren_i                 (sram_wren),
        .waddr_i                (sram_waddr),
        .wbyteenable_i          ({(DW*SIZE/8){1'b1}}),
        .wdata_i                (sram_wdata),
        .raddr_i                (sram_raddr),
        .rdata_o                (sram_rdata)
    );
        
    reg     [DW-1:0]            a[SIZE][LENGTH];
    reg     [DW-1:0]            a_shifted[SIZE][LENGTH+SIZE];

    // Initialize the array, write to memory
    // and align the inputs
    initial begin
        // initialize the array
        for (int i=0; i<SIZE; i++) begin
            for (int j=0; j<LENGTH; j++) begin
                //a[i][j]                 = 32'd1;
                a[i][j]                 = (i+j);
            end
        end

        // write to memory
        for (int i=0; i<LENGTH; i++) begin
            u_mem.write(i, {a[0][i], a[1][i], a[2][i], a[3][i]});
        end

        // align the inputs
        for (int i=0; i<SIZE; i++) begin
            // prepare shifted array
            // shifted[0]   :   a[0][0] x       x       x
            // shifted[1]   :   a[0][1] a[1][0] x       x
            // shifted[2]   :   a[0][2] a[1][1] a[2][0] x
            for (int j=0; j<LENGTH+SIZE; j++) begin
                a_shifted[i][j]             = 32'hX;
            end
            for (int j=0; j<LENGTH; j++) begin
                a_shifted[i][j+i]           = a[i][j];
            end
        end
    end

    // main
    initial begin
        // drive reset values
        start                   = 1'b0;

        // wait for a reset release
        repeat (100) @(posedge clk);

        // start
        start                   = 1'b1;
        @(posedge clk);
        start                   = 1'b0;
        @(posedge clk);

        // compare
        for (int j=0; j<LENGTH+SIZE; j++) begin
            for (int i=0; i<SIZE; i++) begin
                $display("(%d, %d) %x(real), %x(expected)", i, j, a_w[i], a_shifted[i][j]);
                if (a_w[i]!=a_shifted[i][j]) begin
                    $display("Mismatch!");
                    @(posedge clk);
                    $finish;
                end
            end
            @(posedge clk);
        end

        // Wait for completion
        while (!done) begin
            @(posedge clk);
        end
        @(posedge clk);

        $finish;
    end

endmodule
