`timescale 1ns/1ps
module tb_fuzzy_clear;
    localparam signed [31:0] Q=65536;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0, tick=0, clear=1;
    reg signed [31:0] target=-48*Q;
    wire signed [31:0] out;
    wire signed [32:0] filtered;
    wire [9:0] pscale,dscale;
    integer n, checks=0;
    fuzzy_pid_q16 #(.E_STEP_SHIFT(4),.EC_STEP_SHIFT(1)) dut(
        .clk(clk),.rst_n(rst_n),.en(tick && !clear),.clear(clear),.fuzzy_enable(1'b1),
        .target(target),.actual(32'sd0),.kp(-32'sd98304),.ki(-32'sd1966),.kd(-32'sd2097152),
        .out_max(750*Q),.out_min(-750*Q),.en_dlpf(1'b1),.en_dspan2(1'b1),
        .en_dspan4(1'b0),.lpf_shift(2'd0),.out(out),
        .kp_scale_dbg(pscale),.kd_scale_dbg(dscale),.ec_filtered_dbg(filtered));
    task step;begin
        @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(40)@(negedge clk);
    end endtask
    task check;input ok;input [8*96-1:0] name;begin
        checks=checks+1;if(ok!==1)$fatal(1,"%0s integ=%0d D=%0d clear_q=%b",
            name,dut.u_pid.integ,dut.u_pid.d_lpf,dut.clear_q);
    end endtask
    initial begin
        repeat(4)@(negedge clk);rst_n=1;repeat(4)@(negedge clk);clear=0;
        repeat(5)step;
        check(dut.u_pid.integ>0,"integral accumulated before stop");
        $display("BEFORE CLEAR integ=%0d D=%0d",dut.u_pid.integ,dut.u_pid.d_lpf);
        @(negedge clk);clear=1;

        repeat(40)@(negedge clk);
        $display("DURING CLEAR integ=%0d D=%0d",dut.u_pid.integ,dut.u_pid.d_lpf);
        check(dut.u_pid.integ==0 && dut.u_pid.d_lpf==0,"clear must reach core without a sample");
        target=-60*Q;clear=0;
        step;
        check(dut.u_pid.integ==0 && dut.u_pid.d_lpf==0,"first resumed sample clears integral and derivative history");
        check(filtered==0 && pscale==256 && dscale==256,"first resumed fuzzy sample has no stale scheduling history");
        step;check(dut.u_pid.integ>0,"integration resumes on next sample");

        @(negedge clk);target=-61*Q;tick=1;
        @(negedge clk);tick=0;repeat(5)@(negedge clk);clear=1;
        repeat(3)@(negedge clk);clear=0;repeat(40)@(negedge clk);
        check(dut.u_pid.integ==0 && dut.u_pid.d_lpf==0,"in-flight sample cannot restore cleared state");
        target=-64*Q;step;
        check(dut.u_pid.d_lpf==0 && filtered==0 && pscale==256 && dscale==256,
            "short clear remains pending until fresh sample");
        $display("ALL PASS fuzzy clear %0d",checks);$finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
