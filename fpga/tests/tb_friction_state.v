`timescale 1ns/1ps
module tb_friction_state;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,tick=0,active=0,enable=0;
    reg signed [15:0] delta=0;
    reg [3:0] shape=0;
    wire [15:0] comp;
    wire [3:0] shift;
    wire moving;
    integer k, last_comp, checks=0;
    friction_state dut(clk,rst_n,tick,active,enable,delta,16'd130,16'd50,shape,comp,shift,moving);
    task check; input ok; begin checks=checks+1; if(ok!==1) $fatal(1,"friction %0d",checks); end endtask
    task step; input signed [15:0] dp; begin
        @(negedge clk); delta=dp; tick=1;
        @(negedge clk); tick=0;
    end endtask
    initial begin
        repeat(3) @(negedge clk); rst_n=1;
        step(0); check(comp==130 && shift==0);
        enable=1; active=1;

        for(k=0;k<20;k=k+1) step(k%5==0 ? 1 : 0);
        check(moving && shift==1);
        last_comp=comp;
        for(k=0;k<60;k=k+1) begin
            step(k%5==0 ? -1 : 0);
            check(last_comp-comp>=0 && last_comp-comp<=8); last_comp=comp;

            if(k==9) check(comp==50);
        end
        check(comp==50 && moving);
        for(k=0;k<40;k=k+1) step(0);
        check(moving);
        for(k=0;k<20;k=k+1) step(0);
        check(!moving);
        for(k=0;k<40;k=k+1) step(0);
        check(comp==130);
        enable=0; step(0); check(comp==130 && shift==0 && !moving);

        @(negedge clk); shape=0; enable=1; #1; check(shift==1);
        @(negedge clk); shape=1;          #1; check(shift==2);
        @(negedge clk); shape=2;          #1; check(shift==3);
        @(negedge clk); shape=3;          #1; check(shift==3);
        @(negedge clk); shape=4'ha;       #1; check(shift==3);

        @(negedge clk); shape=0; enable=0; #1; check(shift==0);
        @(negedge clk); shape=2;           #1; check(shift==2);
        @(negedge clk); shape=3;           #1; check(shift==3);
        $display("ALL PASS friction %0d",checks); $finish;
    end
endmodule
