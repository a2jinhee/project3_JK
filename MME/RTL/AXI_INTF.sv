// Copyright (c) 2021 Sungkyunkwan University
//
// Authors:
// - Jungrae Kim <dale40@skku.edu>

`include "../../RTL/AXI_TYPEDEF.svh"

interface AXI_AW_CH
#(
    parameter   ADDR_WIDTH      = `AXI_ADDR_WIDTH,
    parameter   ID_WIDTH        = `AXI_ID_WIDTH
 )
(
    input                       clk
);
    logic                       awvalid;
    logic                       awready;
    logic   [ID_WIDTH-1:0]      awid;
    logic   [ADDR_WIDTH-1:0]    awaddr;
    logic   [3:0]               awlen;
    logic   [2:0]               awsize;
    logic   [1:0]               awburst;

    modport master (
        output          awvalid, awid, awaddr, awlen, awsize, awburst,
        input           awready
    );

endinterface

interface AXI_W_CH
#(
    parameter   DATA_WIDTH      = `AXI_DATA_WIDTH,
    parameter   ID_WIDTH        = `AXI_ID_WIDTH
 )
(
    input                       clk
);
    logic                       wvalid;
    logic                       wready;
    logic   [ID_WIDTH-1:0]      wid;
    logic   [DATA_WIDTH-1:0]    wdata;
    logic   [DATA_WIDTH/8-1:0]  wstrb;
    logic                       wlast;

    modport master (
        output          wvalid, wid, wdata, wstrb, wlast,
        input           wready
    );

endinterface

interface AXI_B_CH
#(
    parameter   ID_WIDTH        = `AXI_ID_WIDTH
 )
(
    input                       clk
);
    logic                       bvalid;
    logic                       bready;
    logic   [ID_WIDTH-1:0]      bid;
    logic   [1:0]               bresp;

    modport slave (
        input           bvalid, bid, bresp,
        output          bready
    );

endinterface

interface AXI_AR_CH
#(
    parameter   ADDR_WIDTH      = `AXI_ADDR_WIDTH,
    parameter   ID_WIDTH        = `AXI_ID_WIDTH
 )
(
    input                       clk
);
    logic                       arvalid;
    logic                       arready;
    logic   [ID_WIDTH-1:0]      arid;
    logic   [ADDR_WIDTH-1:0]    araddr;
    logic   [3:0]               arlen;
    logic   [2:0]               arsize;
    logic   [1:0]               arburst;

    modport master (
        output          arvalid, arid, araddr, arlen, arsize, arburst,
        input           arready
    );

endinterface

interface AXI_R_CH
#(
    parameter   DATA_WIDTH      = `AXI_DATA_WIDTH,
    parameter   ID_WIDTH        = `AXI_ID_WIDTH
 )
(
    input                       clk
);
    logic                       rvalid;
    logic                       rready;
    logic   [ID_WIDTH-1:0]      rid;
    logic   [DATA_WIDTH-1:0]    rdata;
    logic   [1:0]               rresp;
    logic                       rlast;

    modport slave (
        input           rvalid, rid, rdata, rresp, rlast,
        output          rready
    );

endinterface

interface APB (
    input                       clk
);
    logic                       psel;
    logic                       penable;
    logic   [31:0]              paddr;
    logic                       pwrite;
    logic   [31:0]              pwdata;
    logic                       pready;
    logic   [31:0]              prdata;
    logic                       pslverr;

    modport master (
        input           clk,
        input           pready, prdata, pslverr,
        output          psel, penable, paddr, pwrite, pwdata
    );

    modport slave (
        output          pready, prdata, pslverr,
        input           psel, penable, paddr, pwrite, pwdata
    );

    // synthesis translate_off
    // a semaphore to allow only one access at a time
    semaphore                   sema;
    initial begin
        sema                        = new(1);
    end

    task init();
        psel                    = 1'b0;
        penable                 = 1'b0;
        paddr                   = 32'd0;
        pwrite                  = 1'b0;
        pwdata                  = 32'd0;
    endtask

    task automatic write(input int addr,
                         input int data);
        // during a write, another threads cannot access APB
        sema.get(1);
        #1
        psel                    = 1'b1;
        penable                 = 1'b0;
        paddr                   = addr;
        pwrite                  = 1'b1;
        pwdata                  = data;
        @(posedge clk);
        #1
        penable                 = 1'b1;
        @(posedge clk);

        while (pready==1'b0) begin
            @(posedge clk);
        end

        psel                    = 1'b0;
        penable                 = 1'b0;
        paddr                   = 'hX;
        pwrite                  = 1'bx;
        pwdata                  = 'hX;

        // release the semaphore
        sema.put(1);
    endtask

    task automatic read(input int addr,
                        output int data);
        // during a read, another threads cannot access APB
        sema.get(1);

        #1
        psel                    = 1'b1;
        penable                 = 1'b0;
        paddr                   = addr;
        pwrite                  = 1'b0;
        pwdata                  = 'hX;
        @(posedge clk);
        #1
        penable                 = 1'b1;
        @(posedge clk);

        while (pready==1'b0) begin
            @(posedge clk);
        end
        data                    = prdata;

        psel                    = 1'b0;
        penable                 = 1'b0;
        paddr                   = 'hX;
        pwrite                  = 1'bx;
        pwdata                  = 'hX;

        // release the semaphore
        sema.put(1);
    endtask
    // synthesis translate_on

endinterface
