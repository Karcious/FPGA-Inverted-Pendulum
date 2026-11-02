`timescale 1ns/1ps
module tb_key_rebase_collision;
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
        positive_key;repeat(12)ms;actual_pos=12;dpos=0;repeat(3)ms;
        check(dut.key_arrival);
        key_pos=1;
        @(posedge clk);#1;actual_pos=0;
        check(ref_pos==0&&cmd_pos==0&&dut.pending_key==2&&ready);
        @(negedge clk);key_pos=0;
        @(posedge clk);#1;
        check(ref_pos==0&&cmd_pos==(12<<16)&&traveling);
        @(negedge clk);
        check(dut.pending_key==0);
        dut.key_arrival=1;profile_request=1;profile_target=5<<16;profile_slew=1136;
        @(posedge clk);#1;
        check(ref_pos==0&&cmd_pos==(5<<16)&&traveling);
        @(negedge clk);profile_request=0;re_zero=1;key_neg=1;
        @(posedge clk);#1;check(ref_pos==0&&dut.pending_key==1);
        @(negedge clk);re_zero=0;key_neg=0;actual_pos=0;repeat(12)ms;
        check(cmd_pos==-(12<<16)&&traveling);
        $display("ALL PASS key rebase collision %0d",checks);$finish;
    end
endmodule
