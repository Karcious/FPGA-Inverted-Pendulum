`timescale 1ns/1ps
module tb_motion_priority;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,tick=0,active=1,confirmed=1,re_zero=0,fault=0;
    reg key_neg=0,key_zero=0,key_pos=0,hold_motion=0,profile_request=0;
    reg signed [15:0] angle_error=0,actual_pos=0,dpos=0;
    reg signed [31:0] profile_target=0,profile_slew=0;
    wire signed [31:0] ref_pos,cmd_pos;
    wire ready,moving,paused,reached,homing,home_fault,at_target,traveling,recovering;
    integer checks=0,k;
    reg signed [31:0] saved_ref;
    key_motion_target #(.READY_TICKS(3),.REACHED_TICKS(4),.KEY_SETTLE_TICKS(3),
        .STEP_Q16(12<<16),.SLEW_Q16(1<<16)) dut(
        .clk(clk),.rst_n(rst_n),.tick(tick),.active(active),
        .balance_confirmed(confirmed),.re_zero(re_zero),.fault(fault),
        .key_neg(key_neg),.key_zero(key_zero),.key_pos(key_pos),
        .remote_move_req(1'b0),.remote_move_counts(16'sd0),
        .profile_request(profile_request),.profile_target(profile_target),
        .profile_slew(profile_slew),.hold_motion(hold_motion),
        .angle_error(angle_error),.actual_pos(actual_pos),.dpos(dpos),

        .slew_sel(32'sd0),.prof_slew_sel(32'sd0),.accel_sel(32'sd0),.decel_sel(32'sd0),
        .home_slew_sel(32'sd0),.home_leg(),
        .target_ref(ref_pos),.target_cmd(cmd_pos),.ready(ready),
        .moving(moving),.paused(paused),.reached(reached),
        .homing(homing),.home_fault(home_fault),.at_target(at_target),
        .traveling(traveling),.recovering(recovering));
    task check; input ok; begin
        checks=checks+1;
        if(ok!==1)$fatal(1,"priority %0d ref=%0d cmd=%0d travel=%b recover=%b reached=%b",
            checks,ref_pos,cmd_pos,traveling,recovering,reached);
    end endtask
    task ms; begin
        @(negedge clk);tick=1;@(negedge clk);tick=0;#1;
    end endtask
    task positive_key; begin
        @(negedge clk);key_pos=1;
        @(negedge clk);key_pos=0;#1;
    end endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;repeat(12)ms;
        check(ready && reached && !traveling);

        angle_error=100;repeat(3)ms;positive_key;
        for(k=0;k<12;k=k+1)begin
            ms;check(traveling && !recovering && !paused && ref_pos==((k+1)<<16));
        end
        check(!moving && traveling && !reached);
        actual_pos=6;dpos=2;ms;check(traveling && !at_target);
        dpos=1;ms;check(traveling && !reached && !at_target);
        dpos=0;repeat(4)ms;
        check(!traveling && !reached && !at_target);
        actual_pos=0;dpos=0;repeat(8)ms;
        check(!reached);
        angle_error=0;repeat(4)ms;check(reached && at_target);

        @(negedge clk);tick=1;key_neg=1;saved_ref=ref_pos;
        @(negedge clk);tick=0;key_neg=0;#1;
        check(traveling && ref_pos==saved_ref && cmd_pos==-(12<<16) && !reached);
        angle_error=240;saved_ref=ref_pos;ms;
        check(recovering && paused && ref_pos==saved_ref);
        angle_error=200;repeat(3)ms;
        check(recovering && ref_pos==saved_ref);
        angle_error=160;repeat(2)ms;
        check(!recovering && !paused && ref_pos<saved_ref);

        actual_pos=ref_pos[31:16];angle_error=150;ms;saved_ref=ref_pos;
        angle_error=161;ms;
        check(recovering && ref_pos==saved_ref);
        angle_error=150;repeat(2)ms;
        check(!recovering && ref_pos<saved_ref);

        actual_pos=-100;saved_ref=ref_pos;ms;
        check(paused && !recovering && ref_pos==saved_ref && traveling);

        actual_pos=-12;ms;check(traveling);

        angle_error=-32768;saved_ref=ref_pos;ms;
        check(recovering && ref_pos==saved_ref);
        angle_error=32767;ms;check(recovering && ref_pos==saved_ref);

        @(negedge clk);hold_motion=1;
        @(negedge clk);hold_motion=0;#1;
        check(!traveling && !recovering && cmd_pos==ref_pos && !reached);
        angle_error=0;actual_pos=ref_pos[31:16];repeat(3)ms;

        @(negedge clk);profile_request=1;profile_target=(10<<16);profile_slew=1136;
        @(negedge clk);profile_request=0;#1;
        check(traveling && dut.current_slew==1136);

        @(negedge clk);re_zero=1;actual_pos=0;
        @(negedge clk);re_zero=0;#1;
        check(!traveling && !recovering && ref_pos==0 && cmd_pos==0);
        repeat(12)ms;positive_key;check(traveling);
        fault=1;ms;check(!traveling && !ready && !recovering);
        fault=0;repeat(12)ms;positive_key;confirmed=0;ms;
        check(!traveling && !ready && !recovering);
        confirmed=1;repeat(12)ms;positive_key;active=0;ms;
        check(!traveling && !recovering && !ready);
        $display("ALL PASS motion_priority checks=%0d",checks);$finish;
    end
endmodule
