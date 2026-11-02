`timescale 1ns/1ps
module tb_motion_ramp_pause;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,tick=0,key_pos=0,request=0;
    reg signed [15:0] angle=0,actual=0;
    reg signed [31:0] speed=512,accel=128,decel=128;
    wire signed [31:0] ref_pos,cmd_pos;
    wire ready,recovering;
    integer i,checks=0;reg signed[31:0] previous;
    key_motion_target #(.READY_TICKS(1),.REACHED_TICKS(1)) dut(
        .clk(clk),.rst_n(rst_n),.tick(tick),.active(1'b1),.balance_confirmed(1'b1),
        .re_zero(1'b0),.fault(1'b0),.key_neg(1'b0),.key_zero(1'b0),.key_pos(key_pos),
        .remote_move_req(1'b0),.remote_move_counts(16'sd0),
        .profile_request(request),.profile_target(10<<16),.profile_slew(speed),.hold_motion(1'b0),
        .angle_error(angle),.actual_pos(actual),.dpos(16'sd0),.slew_sel(32'sd8192),
        .prof_slew_sel(32'sd3787),.accel_sel(accel),.decel_sel(decel),.home_slew_sel(32'sd0),
        .target_ref(ref_pos),.target_cmd(cmd_pos),.ready(ready),.recovering(recovering));
    task check;input ok;input[8*80-1:0] label;begin
        checks=checks+1;if(ok!==1)$fatal(1,"ramp coordination: %0s",label);
    end endtask
    task ms;begin
        @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(4)@(negedge clk);
    end endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;repeat(8)ms;check(ready,"ready");
        @(negedge clk);request=1;@(negedge clk);request=0;
        for(i=0;i<100;i=i+1)begin
            previous=ref_pos;actual=ref_pos[31:16];ms;
            check(ref_pos-previous<=512,"crawl must not exceed a slow profile's requested speed");
        end
        @(negedge clk);key_pos=1;@(negedge clk);key_pos=0;
        for(i=0;i<100;i=i+1)begin actual=ref_pos[31:16];ms;end
        check(dut.slew_applied>2048,"cruise reached before interruption");
        angle=240;previous=ref_pos;repeat(20)ms;
        check(recovering && ref_pos==previous,"safety pause still freezes reference immediately");
        check(dut.slew_applied<=2048,"paused trajectory must not retain a hidden cruise velocity");
        angle=0;repeat(2)ms;
        check(ref_pos-previous<=2048,"resume begins at crawl instead of jumping to cruise");
        previous=ref_pos;ms;
        check(ref_pos-previous<=2176,"resumed acceleration is bounded");
        $display("ALL PASS motion ramp pause %0d",checks);$finish;
    end
endmodule
