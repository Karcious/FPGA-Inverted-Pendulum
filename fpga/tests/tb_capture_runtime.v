`timescale 1ns/1ps
module tb_capture_runtime;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,en=0,arm=0,disarm=0;
    reg [31:0] log2=2;
    reg [63:0] data=0;
    wire armed,done;
    wire [2:0] idx;
    reg [2:0] address=0;
    reg [2:0] pre=0;
    wire [63:0] result;
    integer k, checks=0;
    capture #(.DEPTH(8),.AW(3),.RUNTIME_AVERAGE(1)) dut (
        .clk(clk),.rst_n(rst_n),.sample_en(en),.sample_data(data),.average_log2(log2),
        .trig_ch(2'd0),.trig_level(16'sd0),.trig_edge(2'd2),.pre_len(pre),
        .arm(arm),.disarm(disarm),.armed(armed),.done(done),.trig_idx(idx),.rd_addr(address),.rd_data(result)
    );
    task check; input ok; begin checks=checks+1; if(ok!==1) $fatal(1,"capture runtime %0d",checks); end endtask
    task step; input integer value; begin
        @(negedge clk); data={16'd0,16'd0,16'd0,value[15:0]};en=1;
        @(negedge clk);en=0;
    end endtask
    initial begin
        repeat(3) @(negedge clk);rst_n=1;
        @(negedge clk);arm=1;@(negedge clk);arm=0;

        log2=0;
        for(k=0;k<36;k=k+1) step(k);
        check(done && !armed);
        repeat(3) @(negedge clk); check(result[15:0]==5);
        address=7;repeat(3) @(negedge clk);check(result[15:0]==33);
        for(k=0;k<40;k=k+1) step(1000);
        check(result[15:0]==33);
        pre=7;log2=0;@(negedge clk);arm=1;@(negedge clk);arm=0;
        for(k=0;k<7;k=k+1)step(k);
        check(!done);
        step(7);check(done);
        address=0;repeat(3)@(negedge clk);check(result[15:0]==0);
        address=7;repeat(3)@(negedge clk);check(result[15:0]==7);
        pre=0;log2=2;@(negedge clk);arm=1;@(negedge clk);arm=0;
        for(k=0;k<36;k=k+1)step(k-36);
        check(done);
        address=0;repeat(3)@(negedge clk);check($signed(result[15:0])==-31);
        $display("ALL PASS capture runtime %0d",checks);$finish;
    end
endmodule
