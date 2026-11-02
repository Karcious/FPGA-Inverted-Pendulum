`timescale 1ns / 1ps

`include "dbg_pkg.vh"

module tb_fuzzy_telemetry;

    localparam integer CLK_FREQ = 1_000_000;
    localparam integer BAUD     = 100_000;
    localparam integer BIT_T    = CLK_FREQ / BAUD;
    localparam integer DEPTH    = 64;
    localparam integer AW       = 6;
    localparam integer NCH      = 4;
    localparam integer DW       = 16;

    localparam integer N_PARAM  = 47;

    localparam integer PRE_LEN  = 4;
    localparam integer TRIG_CH  = 1;
    localparam integer TRIG_LVL = 500;
    localparam integer TRIG_IDX = TRIG_LVL / 100;
    localparam integer N_SAMPLE = DEPTH + 1;

    reg               clk    = 1'b0;
    reg               rst_n  = 1'b0;
    reg               rx_pin = 1'b1;
    wire              tx_pin;
    wire [N_PARAM*32-1:0] param_flat;
    reg               sample_en = 1'b0;
    reg  [NCH*DW-1:0] probe_data = {(NCH*DW){1'b0}};
    wire              cap_armed, cap_done;
    reg [31:0] live_status=32'h12345684;
    reg [127:0] fuzzy_data=0;
    reg [N_PARAM*32-1:0] saved_parameters;

    always #500 clk = ~clk;

    dbg_top #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD),
        .NCH      (NCH),
        .DW       (DW),
        .DEPTH    (DEPTH),
        .AW       (AW),
        .N_PARAM  (N_PARAM)

    ) dut (
        .control_status(live_status),
        .fuzzy_debug_data(fuzzy_data),
        .clk          (clk),
        .rst_n        (rst_n),
        .uart_rx_pin  (rx_pin),
        .uart_tx_pin  (tx_pin),
        .param_flat   (param_flat),
        .sample_en    (sample_en),
        .probe_data   (probe_data),
        .cap_armed    (cap_armed),
        .cap_done     (cap_done)
    );

    integer errors = 0;
    integer checks = 0;

    task check;
        input        cond;
        input [255:0] name;
        begin
            checks = checks + 1;
            if (cond !== 1'b1) begin
                errors = errors + 1;
                $display("  [FAIL] %0s", name);
            end
        end
    endtask

    function [7:0] crcstep;
        input [7:0] c;
        input [7:0] d;
        integer k;
        reg [7:0] x;
        begin
            x = c ^ d;
            for (k = 0; k < 8; k = k + 1)
                x = x[7] ? ((x << 1) ^ 8'h07) : (x << 1);
            crcstep = x;
        end
    endfunction

    task uart_put;
        input [7:0] b;
        integer i;
        begin
            rx_pin = 1'b0;
            repeat (BIT_T) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                rx_pin = b[i];
                repeat (BIT_T) @(posedge clk);
            end
            rx_pin = 1'b1;
            repeat (BIT_T) @(posedge clk);
        end
    endtask

    integer txlog;

    task uart_get;
        output [7:0] b;
        integer i;
        begin
            @(negedge tx_pin);
            repeat (BIT_T + BIT_T/2) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = tx_pin;
                if (i != 7) repeat (BIT_T) @(posedge clk);
            end
            repeat (BIT_T) @(posedge clk);
            $fwrite(txlog, "%02x\n", b);
        end
    endtask

    reg [7:0] txpay [0:1023];
    reg [7:0] rxpay [0:1023];

    task send_frame;
        input [7:0]  cmd;
        input [15:0] len;
        integer k;
        reg [7:0] c;
        begin
            c = 8'h00;
            uart_put(8'hAA);
            uart_put(8'h55);
            uart_put(cmd);       c = crcstep(c, cmd);
            c = crcstep(c, len[7:0]);   uart_put(len[7:0]);
            c = crcstep(c, len[15:8]);  uart_put(len[15:8]);
            for (k = 0; k < len; k = k + 1) begin
                uart_put(txpay[k]);
                c = crcstep(c, txpay[k]);
            end
            uart_put(c);
        end
    endtask

    task recv_frame;
        output [7:0]  cmd;
        output [15:0] len;
        integer k;
        reg [7:0] b, c, c0, c1;
        begin
            uart_get(b); check(b === 8'hAA, "resp magic0");
            uart_get(b); check(b === 8'h55, "resp magic1");
            uart_get(b);
            cmd = b;
            c = crcstep(8'h00, b);
            uart_get(c0); c = crcstep(c, c0);
            uart_get(c1); c = crcstep(c, c1);
            len = {c1, c0};
            for (k = 0; k < len; k = k + 1) begin
                uart_get(b);
                rxpay[k] = b;
                c = crcstep(c, b);
            end
            uart_get(b);
            check(b === c, "resp crc8");
        end
    endtask

    reg [7:0]  rcmd;
    reg [15:0] rlen;
    reg [31:0] v;

    task get_param;
        input [5:0] addr;
        input [31:0] want;
        begin
            txpay[0] = {2'b0, addr};
            send_frame(`CMD_GET_PARAM, 1);
            recv_frame(rcmd, rlen);
            check(rcmd === (`CMD_GET_PARAM | `DBG_RESP_FLAG), "GET_PARAM resp cmd");
            check(rlen === 16'd5, "GET_PARAM resp len");
            check(rxpay[0] === {2'b0, addr}, "GET_PARAM echo addr");
            v = {rxpay[4], rxpay[3], rxpay[2], rxpay[1]};
            check(v === want, "GET_PARAM value");
            if (v !== want) $display("      want %08x got %08x", want, v);
        end
    endtask

    task set_param;
        input [5:0]  addr;
        input [31:0] val;
        begin
            txpay[0] = {2'b0, addr};
            txpay[1] = val[7:0];
            txpay[2] = val[15:8];
            txpay[3] = val[23:16];
            txpay[4] = val[31:24];
            send_frame(`CMD_SET_PARAM, 5);
            recv_frame(rcmd, rlen);
            check(rcmd === (`CMD_SET_PARAM | `DBG_RESP_FLAG), "SET_PARAM resp cmd");
            check(rxpay[0] === {2'b0, addr}, "SET_PARAM echo addr");
        end
    endtask

    task push_sample;
        input integer i;
        begin
            probe_data[(0*DW) +: DW] = i[15:0];
            probe_data[(1*DW) +: DW] = i * 100;
            probe_data[(2*DW) +: DW] = 16'h1000 + i[15:0];
            probe_data[(3*DW) +: DW] = 16'h2000 + i[15:0];
            @(posedge clk);
            sample_en = 1'b1;
            @(posedge clk);
            sample_en = 1'b0;
        end
    endtask

    integer i, base;
    reg [15:0] w;

    initial begin
        txlog=$fopen("fuzzy_tx.hex","w");
        repeat(20) @(negedge clk);
        rst_n=1;
        repeat(5) @(negedge clk);
        saved_parameters=param_flat;
        fuzzy_data=128'h00140901_ffff8000_00640000_00038000;
        sample_en=1;
        @(negedge clk); sample_en=0;
        repeat(5) @(negedge clk);
        fork
            begin send_frame(`CMD_READ_FUZZY,0); recv_frame(rcmd,rlen); end
            begin
                wait(dut.u_proto.resp_req);
                @(negedge clk);
                fuzzy_data=128'h00040100_00000000_00020000_00010000;
                sample_en=1;
                @(negedge clk); sample_en=0;
            end
        join
        check(rcmd===8'h8a && rlen===16,"fuzzy command and length");
        for(i=0;i<16;i=i+1)
            check({120'd0,rxpay[i]} === (128'h00140901_ffff8000_00640000_00038000 >> (i*8) & 8'hff),
                  "atomic little endian snapshot");
        check(param_flat===saved_parameters,"fuzzy read preserves every parameter");
        check(cap_armed===0,"fuzzy read leaves capture idle");
        send_frame(`CMD_READ_FUZZY,0); recv_frame(rcmd,rlen);
        for(i=0;i<16;i=i+1)
            check({120'd0,rxpay[i]} === (128'h00040100_00000000_00020000_00010000 >> (i*8) & 8'hff),
                  "next snapshot reflects new input");
        check(param_flat===saved_parameters,"second read preserves parameters");
        if(errors==0) $display("ALL PASS fuzzy telemetry checks=%0d",checks);
        else $display("FAILED: %0d",errors);
        $fclose(txlog); $finish;
    end

    initial begin
        #500_000_000;
        $display("*** TIMEOUT ***");
        $finish;
    end

endmodule
