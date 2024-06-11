// Copyright (c) 2021 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

`define 	TIMEOUT_DELAY 	99999999

module SYSTOLIC_ARRAY_TB ();

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

    localparam                  DW      = 32;
    localparam                  SIZE    = 4;
    localparam  reg [7:0]       LENGTH  = 8'd8;

    reg                         start;
    wire                        done;
    reg     [DW-1:0]            a[SIZE][LENGTH];
    reg     [DW-1:0]            b[LENGTH][SIZE];
    reg     [DW-1:0]            a_shifted[SIZE][LENGTH+SIZE];
    reg     [DW-1:0]            b_shifted[LENGTH+SIZE][SIZE];
    wire    [2*DW:0]            accum[SIZE][SIZE];
    int                         k;

    SYSTOLIC_ARRAY
    #(
        .DW                     (DW),
        .SA_WIDTH               (SIZE)
    )
    u_DUT
    (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .mat_width_i            (LENGTH),
        .start_i                (start),
        .done_o                 (done),
        .a_i                    ({a_shifted[0][k], a_shifted[1][k], a_shifted[2][k], a_shifted[3][k]}),
        .b_i                    ({b_shifted[k][0], b_shifted[k][1], b_shifted[k][2], b_shifted[k][3]}),
        .accum_o                (accum)
    );

    // Initialize the array, generate expected output,
    // and align the inputs
    reg     [2*DW:0]            expected_accum[SIZE][SIZE];
    initial begin
        // initialize the array
        for (int i=0; i<SIZE; i++) begin
            for (int j=0; j<LENGTH; j++) begin
                //a[i][j]                 = 32'd1;
                //b[j][i]                 = 32'd1;
                a[i][j]                 = (i+j);
                b[j][i]                 = (i+j);
            end
        end

        // generate expected output
        for (int i=0; i<SIZE; i++) begin
            for (int j=0; j<SIZE; j++) begin
                expected_accum[i][j] = 0;
                for (int k=0; k<LENGTH; k++) begin
                    expected_accum[i][j] += a[i][k] * b[k][j];
                end
            end
        end

        // align the inputs
        for (int i=0; i<SIZE; i++) begin
            // prepare shifted array
            // shifted[0]   :   a[0][0] x       x       x
            // shifted[1]   :   a[0][1] a[1][0] x       x
            // shifted[2]   :   a[0][2] a[1][1] a[2][0] x
            // init
            for (int j=0; j<LENGTH+SIZE; j++) begin
                a_shifted[i][j]             = 32'hX;
                b_shifted[j][i]             = 32'hX;
            end
            for (int j=0; j<LENGTH; j++) begin
                a_shifted[i][j+i]           = a[i][j];
                b_shifted[j+i][i]           = b[j][i];
            end
        end
    end

    // main
    initial begin
        // drive reset values
        start                   = 1'b0;

        k                       = 0;
        // wait for a reset release
        repeat (100) @(posedge clk);

        // start
        start                   = 1'b1;
        @(posedge clk);
        k++;    // increment the index to feed the next data
        start                   = 1'b0;
        @(posedge clk);
        while (!done) begin
            if (k<LENGTH+SIZE) begin
                k++;
            end
            else begin
                k               = 0;
            end
            @(posedge clk);
        end
        @(posedge clk);

        // verify the results
        for (int i=0; i<SIZE; i++) begin
            $write("@%3drow: ", i);
            for (int j=0; j<SIZE; j++) begin
                $write("%8d (%8d)", accum[i][j], expected_accum[i][j]);
                if (accum[i][j]!=expected_accum[i][j]) begin
                    $display("\n Mismatch at (%d, %d): real (%x) vs. expected (%x)",
                            i, j, accum[i][j], expected_accum[i][j]);
                    @(posedge clk);
                    $finish;
                end
            end
            $write("\n");
        end
        $finish;
    end

endmodule
