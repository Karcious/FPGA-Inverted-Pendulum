`timescale 1ns/1ps

module tb_home_ready_window;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,tick=0,active=0,balance_confirmed=0,re_zero=0,fault=0;
    reg key_neg=0,key_zero=0,key_pos=0;
    reg signed [15:0] angle_error=0,actual_pos=0,dpos=0;
    wire signed [31:0] target_ref,target_cmd;
    wire ready,moving,paused,reached,homing,home_fault;
    wire traveling,recovering,home_leg,key_arrival;

    integer checks=0,n,c;
    reg signed [31:0] previous_ref;

    reg arrival_seen;
    always @(posedge clk) if (key_arrival) arrival_seen<=1'b1;

    key_motion_target dut(
        .clk(clk),.rst_n(rst_n),.tick(tick),.active(active),
        .balance_confirmed(balance_confirmed),.re_zero(re_zero),.fault(fault),
        .key_neg(key_neg),.key_zero(key_zero),.key_pos(key_pos),
        .remote_move_req(1'b0),.remote_move_counts(16'sd0),
        .profile_request(1'b0),.profile_target(32'sd0),.profile_slew(32'sd0),
        .hold_motion(1'b0),
        .angle_error(angle_error),.actual_pos(actual_pos),.dpos(dpos),
        .slew_sel(32'sd0),.prof_slew_sel(32'sd0),.accel_sel(32'sd0),.decel_sel(32'sd0),
        .home_slew_sel(32'sd0),
        .target_ref(target_ref),.target_cmd(target_cmd),.ready(ready),
        .moving(moving),.paused(paused),.reached(reached),.homing(homing),
        .home_fault(home_fault),.at_target(),.traveling(traveling),
        .recovering(recovering),.home_leg(home_leg),.key_arrival(key_arrival));

    task check; input ok; input [1023:0] tag;
        begin checks=checks+1;
            if(ok!==1) $fatal(1,"pending %0d [%0s] key=%0d age=%0d ref=%0d cmd=%0d ready=%b",
                checks,tag,dut.pending_key,dut.pending_age,target_ref,target_cmd,ready);
        end
    endtask
    task mark; input [1023:0] s; begin
        $display("== %0s  (checks=%0d t=%0t)", s, checks, $time);
    end endtask
    task ms; begin
        @(negedge clk); tick=1; @(negedge clk); tick=0; #1;
    end endtask
    task keys; input neg,zero,pos; begin
        @(negedge clk); key_neg=neg;key_zero=zero;key_pos=pos;
        @(negedge clk);key_neg=0;key_zero=0;key_pos=0;#1;
    end endtask

    task hard_reset; begin
        rst_n=0; active=0; balance_confirmed=0; re_zero=0; fault=0;
        key_neg=0; key_zero=0; key_pos=0;
        angle_error=0; actual_pos=0; dpos=0;
        repeat(3)@(negedge clk); rst_n=1; #1;
    end endtask

    task bringup; input signed [15:0] pos; begin
        actual_pos=pos; ms; active=1; ms; balance_confirmed=1; ms;
    end endtask
    task follow_ramp; input signed [31:0] goal; input integer timeout;
        integer count;
        begin
            count=0;
            while(target_ref!=goal) begin
                actual_pos=target_ref[31:16];ms;
                count=count+1;
                if(count>timeout)$fatal(1,"pending: ramp timeout");
            end
            $display("   ramp goal=%0d steps=%0d", goal, count);
            actual_pos=goal[31:16];n=count;
        end
    endtask

    task until_ready;
        begin
            c=0;
            while(!ready && c<2000) begin ms; c=c+1; end
            check(ready,"until_ready");
        end
    endtask

    task wait_fire; input signed [31:0] want;
        begin
            c=0;
            while(target_cmd!=want && c<8) begin @(negedge clk);#1; c=c+1; end
            check(target_cmd==want,"wait_fire");
        end
    endtask

    task await_arrival;
        begin
            c=0; arrival_seen=0;
            while(!arrival_seen && c<80) begin ms; c=c+1; end
            check(arrival_seen,"arrival happened");
            ms;
            check(target_ref==0&&target_cmd==0,"targets zeroed");
            check(!key_arrival,"arrival is one shot");
            check(ready,"ready untouched");
            check(dut.qualified,"qualified untouched");
            check(dut.pending_key==2'd0,"nothing pending after arrival");
            actual_pos=0; #1;
        end
    endtask

    initial begin
        hard_reset;bringup(6);repeat(1200)ms;
        check(ready && !reached,"six-count residual unlocks keys without false precise-arrival");
        hard_reset;bringup(7);repeat(1200)ms;
        check(!ready,"seven-count residual remains outside home window");
        hard_reset;angle_error=61;bringup(0);repeat(1200)ms;
        check(!ready,"angle gate remains enforced");
        hard_reset;dpos=2;bringup(0);repeat(1200)ms;
        check(!ready,"velocity gate remains enforced");
        hard_reset;bringup(0);repeat(300)ms;
        repeat(150)ms;dpos=2;ms;dpos=0;repeat(150)ms;
        check(!ready,"stability dwell must be consecutive");
        repeat(60)ms;check(ready,"200ms continuous stability unlocks");
        $display("ALL PASS home ready window %0d",checks);$finish;
    end
endmodule
