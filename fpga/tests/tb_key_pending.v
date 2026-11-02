`timescale 1ns/1ps

module tb_key_pending;
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

        hard_reset; bringup(16'sd0);
        repeat(100)ms;
        check(!ready,"1 no ready yet");
        keys(0,0,1);
        check(target_cmd==0,"1 key does not move while !ready");
        check(dut.pending_key==2'd2,"1 latched as SW4");
        check(dut.pending_age==16'd2000,"1 fresh age");
        mark("1 == key latched while !ready");
        until_ready;
        check(dut.pending_key==2'd2,"1 still latched at ready");
        wait_fire(32'sd17039360);
        check(dut.pending_key==2'd0&&dut.pending_age==16'd0,"1 consumed");
        check(moving&&!reached,"1 now moving");
        follow_ramp(32'sd17039360,2100);check(n==2080,"1 90 deg = 2080 ms");
        mark("1 == pending replayed: +90 deg travel");

        await_arrival;
        check(!reached,"7 reached cleared by the move");
        check(dut.qualified&&ready,"7 still qualified and ready");
        mark("7 == arrival: zero moved, ready/qualified kept");
        keys(0,0,1);
        check(target_cmd==17039360&&moving,"7 next key moves at once");
        follow_ramp(32'sd17039360,2100);check(n==2080,"7 another 90 deg = 2080 ms");
        await_arrival;

        hard_reset; bringup(16'sd0);
        angle_error=16'sd61;
        repeat(10)ms;check(!ready,"2 held not ready");
        keys(0,0,1);
        check(dut.pending_key==2'd2,"2 latched");
        repeat(1900)ms;
        check(dut.pending_key==2'd2,"2 still alive before the deadline");
        repeat(200)ms;
        check(dut.pending_key==2'd0,"2 expired and dropped");
        mark("2 == pending expired");
        angle_error=16'sd0;
        until_ready;
        repeat(5)ms;
        check(target_cmd==0&&!moving,"2 expired pending is never replayed");
        check(dut.pending_key==2'd0,"2 nothing pending");

        hard_reset; bringup(16'sd0);
        angle_error=16'sd61;
        repeat(10)ms;
        keys(0,0,1);
        check(dut.pending_key==2'd2,"3 first is SW4");
        repeat(1500)ms;
        keys(1,0,0);
        check(dut.pending_key==2'd1,"3 later key wins");
        check(dut.pending_age==16'd2000,"3 age refreshed");
        repeat(1900)ms;
        check(dut.pending_key==2'd1,"3 refreshed pending is still alive");
        repeat(200)ms;
        check(dut.pending_key==2'd0,"3 expired 2100 ms after the last press");

        hard_reset; bringup(16'sd0);
        angle_error=16'sd61;
        repeat(10)ms;
        keys(0,0,1);
        check(dut.pending_key==2'd2,"4 latched");
        balance_confirmed=0;ms;
        check(dut.pending_key==2'd0,"4 cleared by !balance_confirmed");
        balance_confirmed=1;
        angle_error=16'sd0;until_ready;repeat(5)ms;
        check(target_cmd==0&&!moving,"4 knock-down key is never replayed");

        hard_reset; bringup(16'sd0);
        angle_error=16'sd61;
        repeat(10)ms;
        keys(0,0,1);
        check(dut.pending_key==2'd2,"5 latched");
        fault=1;ms;
        check(dut.pending_key==2'd0,"5 cleared by fault");
        fault=0;
        angle_error=16'sd0;until_ready;repeat(5)ms;
        check(target_cmd==0&&!moving,"5 fault-time key is never replayed");
        mark("5 == fault also clears pending");

        hard_reset;
        actual_pos=16'sd521;ms;active=1;ms;
        check(home_fault&&!ready,"6 home_fault on activation");
        balance_confirmed=1;
        keys(0,0,1);repeat(10)ms;
        check(home_fault&&target_cmd==(32'sd521<<16),"6 no move during home_fault");
        check(dut.pending_key==2'd0,"6 nothing latched during home_fault");

        repeat(500)ms;
        check(dut.pending_key==2'd0,"6 still nothing latched");

        $display("ALL PASS key_pending checks=%0d",checks);$finish;
    end
endmodule
