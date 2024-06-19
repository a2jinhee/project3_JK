// Copyright (c) 2021 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

`define     OFFSET_IP_VER       32'h000
`define     OFFSET_MAT_CFG      32'h100
`define     OFFSET_MAT_A_ADDR   32'h200
`define     OFFSET_MAT_B_ADDR   32'h204
`define     OFFSET_MAT_C_ADDR   32'h208
`define     OFFSET_MME_CMD      32'h20C
`define     OFFSET_MME_STATUS   32'h210

`define 	TIMEOUT_DELAY 	99999999

module MME_TOP_TB ();

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

    //----------------------------------------------------------
    // Connection between DUT and test modules
    //----------------------------------------------------------
    APB                         apb_if  (.clk(clk));

    AXI_AW_CH                   aw_ch   (.clk(clk));
    AXI_W_CH                    w_ch    (.clk(clk));
    AXI_B_CH                    b_ch    (.clk(clk));
    AXI_AR_CH                   ar_ch   (.clk(clk));
    AXI_R_CH                    r_ch    (.clk(clk));

    MME_TOP  u_DUT (
        .clk                    (clk),
        .rst_n                  (rst_n),

        // APB interface
        .apb_if                 (apb_if),

        // AXI interface
        .axi_aw_if              (aw_ch),
        .axi_w_if               (w_ch),
        .axi_b_if               (b_ch),
        .axi_ar_if              (ar_ch),
        .axi_r_if               (r_ch)
    );

    AXI_SLAVE   u_mem (
        .clk                    (clk),
        .rst_n                  (rst_n),

        .aw_ch                  (aw_ch),
        .w_ch                   (w_ch),
        .b_ch                   (b_ch),
        .ar_ch                  (ar_ch),
        .r_ch                   (r_ch)
    );

    logic   signed [31:0]       mat_a[][];      // 4 x mat_width
    logic   signed [31:0]       mat_b[][];      // mat_width x 4

    //----------------------------------------------------------
    // Testbench starts
    //----------------------------------------------------------

    // For APB write (+ read back the written value to verify)
    task automatic apb_write_n_verify(int addr, int wdata);
        int rdata;

        apb_if.write(addr, wdata);
        apb_if.read(addr, rdata);
        if (rdata!==wdata) begin
            $display("APB write failure @0x%x : Write data = %x, Read-back data = %x", addr, wdata, rdata);
            @(posedge clk);
            $finish;
        end
    endtask

    // initialize the interface
    task init();
        int data;
        apb_if.init();

        @(posedge rst_n);                   // wait for a release of the reset
        repeat (10) @(posedge clk);         // wait another 10 cycles

        apb_if.read(`OFFSET_IP_VER, data);
        $display("---------------------------------------------------");
        $display("IP version: %x", data);
        $display("---------------------------------------------------");
    endtask

    // allocate and initialize input matrixes
    task automatic alloc_and_init_inputs(int mat_width);
        // mat_a[4][mat_width]
        mat_a                   = new[4];
        foreach (mat_a[i])
            mat_a[i]                = new [mat_width];
        // mat_b[mat_width][4]
        mat_b                   = new[mat_width];
        foreach (mat_b[i])
            mat_b[i]                = new [4];

        // initialize data
        for (int row=0; row<4; row++) begin
          for (int col=0; col<mat_width; col++) begin
            mat_a[row][col]                 = $urandom()%256;
          end
        end
        for (int row=0; row<mat_width; row++) begin
          for (int col=0; col<4; col++) begin
            mat_b[row][col]                 = $urandom()%256;
          end
        end
    endtask

    // test matrix multiplication
    task automatic test_mme(int mat_width, int mat_a_addr, int mat_b_addr, int mat_c_addr);
        int data;
        logic signed [64:0] expected_c[4][4];

        $display("---------------------------------------------------");
        $display("Matrix multiplication: A(@0x%x) x B(@0x%x) = C(@0x%x)", mat_a_addr, mat_b_addr, mat_c_addr);
        $display("Matrix sizes         : (4 x %d) x (%d x 4) = (4 x 4)", mat_width, mat_width);
        $display("---------------------------------------------------");

        $display("---------------------------------------------------");
        $display("Load matrix A and B to memory");
        $display("---------------------------------------------------");
        for (int row=0; row<4; row++) begin
            for (int col=0; col<mat_width; col++) begin
                // column-major order
                //   A[0][0] - A[1][0] - A[2][0] - A[3][0] - ...
                // - A[0][1] - A[1][1] - A[2][1] - A[3][1] - ...
                // ...
                int index = col*4+ row;
                // *4 for byte address
                u_mem.write_word(mat_a_addr+index*4, mat_a[row][col]);
            end
        end

        for (int row=0; row<mat_width; row++) begin
            for (int col=0; col<4; col++) begin
                // row-major order
                //   B[0][0] - B[0][1] - B[0][2] - B[0][3]
                // - B[1][0] - B[1][1] - B[1][2] - B[1][3]
                int index = row*4+ col;
                // *4 for byte address
                u_mem.write_word(mat_b_addr+index*4, mat_b[row][col]);
            end
        end

        $display("---------------------------------------------------");
        $display("Configure");
        $display("---------------------------------------------------");
        apb_write_n_verify(`OFFSET_MAT_CFG, mat_width);
        apb_write_n_verify(`OFFSET_MAT_A_ADDR, mat_a_addr);
        apb_write_n_verify(`OFFSET_MAT_B_ADDR, mat_b_addr);
        apb_write_n_verify(`OFFSET_MAT_C_ADDR, mat_c_addr);

        $display("---------------------------------------------------");
        $display("MM start");
        $display("---------------------------------------------------");
        apb_if.write(`OFFSET_MME_CMD, 32'h1);

        data = 0;
        while (data!=1) begin
            apb_if.read(`OFFSET_MME_STATUS, data);
            repeat (10) @(posedge clk);
            $display("Waiting for a MM completion");
        end
        @(posedge clk);
        $display("---------------------------------------------------");
        $display("MM completed");
        $display("---------------------------------------------------");

        $display("---------------------------------------------------");
        $display("Verify result");
        $display("---------------------------------------------------");
        for (int row=0; row<4; row++) begin
            for (int col=0; col<4; col++) begin
                expected_c[row][col]    = 'd0;
                for(int k=0; k<mat_width; k++) begin
                    expected_c[row][col]    += (mat_a[row][k] * mat_b[k][col]);
                end
            end
        end

        for (int row=0; row<4; row++) begin
            for (int col=0; col<4; col++) begin
                // row-major order
                //   C[0][0] - C[0][1] - C[0][2] - C[0][3]
                // - C[1][0] - C[1][1] - C[1][2] - C[1][3]
                int index = row*4+ col;
                // *4 for byte address
                data = u_mem.read_word(mat_c_addr+index*4);
                if (data!==expected_c[row][col][31:0]) begin
                    $display("Output mismatch at (%x, %d): expected=%x, real=%x", row, col, expected_c[row][col], data);
                    @(posedge clk);
                    $finish;
                end
                else begin
                    $display("Output match at (%d, %d)", row, col);
                end
            end
        end

        $display("---------------------------------------------------");
        $display("MM passed");
        $display("---------------------------------------------------");
        $display("");
    endtask

    // main
    initial begin
        int mat_width;

        init();

        //----------------------------------------------------------
        // 1st test
        //----------------------------------------------------------
        // A(4x4) x B(4x4) = C(4x4)
        mat_width               = 4;

        alloc_and_init_inputs(mat_width);

        test_mme(mat_width, 32'h0, 32'h1000, 32'h2000);


        //----------------------------------------------------------
        // 2nd test
        //----------------------------------------------------------
        // A(4x8) x B(8x4) = C(4x4)
        mat_width               = 8;

        alloc_and_init_inputs(mat_width);

        test_mme(mat_width, 32'h0, 32'h1000, 32'h2000);

        //----------------------------------------------------------
        // 3rd test
        //----------------------------------------------------------
        // A(4x12) x B(12x4) = C(4x4)
        mat_width               = 12;

        alloc_and_init_inputs(mat_width);

        test_mme(mat_width, 32'h0, 32'h1000, 32'h2000);


        //----------------------------------------------------------
        // 4th test
        //----------------------------------------------------------
        // A(4x16) x B(16x4) = C(4x4)
        mat_width               = 16;

        alloc_and_init_inputs(mat_width);

        test_mme(mat_width, 32'h0, 32'h1000, 32'h2000);


        $display("Test finished!");
        $finish;
    end

endmodule