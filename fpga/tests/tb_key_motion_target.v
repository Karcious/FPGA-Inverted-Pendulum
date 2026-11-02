`timescale 1ns/1ps
module tb_key_motion_target;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,tick=0,active=0,balance_confirmed=0,re_zero=0,fault=0;
    reg key_neg=0,key_zero=0,key_pos=0;
    reg signed [15:0] angle_error=0,actual_pos=0,dpos=0;
    wire signed [31:0] target_ref,target_cmd;
    wire ready,moving,paused,reached,homing,home_fault;
    wire traveling,recovering;
    wire key_arrival;
    integer checks=0,n;
    reg signed [31:0] previous_ref;

    reg arrival_seen;
    always @(posedge clk) if (key_arrival) arrival_seen<=1'b1;
    key_motion_target dut(
        .clk(clk),.rst_n(rst_n),.tick(tick),.active(active),
        .balance_confirmed(balance_confirmed),.re_zero(re_zero),.fault(fault),
        .key_neg(key_neg),.key_zero(key_zero),.key_pos(key_pos),

        .remote_move_req(1'b0),.remote_move_counts(16'sd0),
        .profile_request(1'b0),.profile_target(32'sd0),.profile_slew(32'sd0),.hold_motion(1'b0),
        .angle_error(angle_error),.actual_pos(actual_pos),.dpos(dpos),

        .slew_sel(32'sd0),.prof_slew_sel(32'sd0),.accel_sel(32'sd0),.decel_sel(32'sd0),

        .home_slew_sel(32'sd0),.home_leg(),
        .target_ref(target_ref),.target_cmd(target_cmd),.ready(ready),
        .moving(moving),.paused(paused),.reached(reached),
        .homing(homing),.home_fault(home_fault),.at_target(),
        .traveling(traveling),.recovering(recovering),
        .key_arrival(key_arrival));
    task check; input ok;
        begin checks=checks+1;
            if(ok!==1) $fatal(1,"motion %0d ref=%0d cmd=%0d ready=%b homing=%b fault=%b",
                checks,target_ref,target_cmd,ready,homing,home_fault);
        end
    endtask
    task ms; begin
        @(negedge clk); tick=1; @(negedge clk); tick=0; #1;
    end endtask
    task keys; input neg,zero,pos; begin
        @(negedge clk); key_neg=neg;key_zero=zero;key_pos=pos;
        @(negedge clk);key_neg=0;key_zero=0;key_pos=0;#1;
    end endtask

    task mark; input [1023:0] s; begin
        $display("== %0s  (checks=%0d t=%0t)", s, checks, $time);
    end endtask
    task follow_ramp; input signed [31:0] goal; input integer timeout;
        integer count;
        begin
            count=0;
            while(target_ref!=goal) begin
                previous_ref=target_ref; actual_pos=target_ref[31:16];ms;
                check((target_ref-previous_ref<=8192)&&(previous_ref-target_ref<=8192));
                if(goal>previous_ref) check(target_ref>=previous_ref && target_ref<=goal);
                else check(target_ref<=previous_ref && target_ref>=goal);
                count=count+1;
                if(count>timeout)$fatal(1,"ramp timeout");
            end
            $display("   ramp goal=%0d steps=%0d", goal, count);
            actual_pos=goal[31:16];n=count;
        end
    endtask

    task await_arrival;
        integer c;
        begin
            c=0; arrival_seen=0;
            while(!arrival_seen && c<80) begin ms; c=c+1; end
            check(arrival_seen);
            ms;
            check(target_ref==0&&target_cmd==0);
            check(!key_arrival);
            check(ready);
            check(dut.qualified);
            actual_pos=0; #1;
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        actual_pos=80;ms;check(target_ref==(80<<16) && !ready);
        keys(0,0,1);check(target_cmd==(80<<16));
        active=1;ms;check(target_ref==(80<<16)&&!home_fault);

        repeat(25)ms;keys(1,0,0);check(target_cmd==(80<<16)&&!ready&&!homing);
        check(dut.pending_key==2'd0);
        balance_confirmed=1;
        angle_error=61;repeat(10)ms;check(!dut.qualified);
        angle_error=-32768;ms;angle_error=60;dpos=2;ms;dpos=-32768;ms;dpos=1;
        repeat(299)ms;check(!dut.qualified&&!ready);

        @(negedge clk);tick=1;key_pos=1;
        @(negedge clk);tick=0;key_pos=0;#1;
        check(homing && moving && !ready && target_cmd==0 && target_ref==(80<<16));
        check(dut.pending_key==2'd2 && dut.pending_age==16'd2000);
        previous_ref=target_ref;angle_error=61;ms;
        check(paused&&target_ref==previous_ref);
        angle_error=-60;actual_pos=63;ms;check(paused&&target_ref==previous_ref);
        actual_pos=64;ms;check(!paused&&target_ref<previous_ref);

        keys(0,0,1);check(target_cmd==0);
        check(dut.pending_key==2'd2 && dut.pending_age==16'd2000);
        angle_error=0;dpos=0;
        follow_ramp(0,2600);check(target_ref==0&&!ready&&!moving);

        check(n>=600 && n<=700);
        repeat(199)ms;check(!ready);
        mark("A == qualification/home/settle done, ready next");
        ms;check(ready&&reached&&!homing);

        check(target_cmd==0 && dut.pending_key==2'd2);
        mark("B ready=1, pending still latched");

        @(negedge clk);#1;
        check(target_cmd==17039360&&moving&&!reached);
        check(dut.pending_key==2'd0&&dut.pending_age==16'd0);
        mark("C == pending replayed: target_cmd=+260, travel starts");
        follow_ramp(17039360,2100);check(n==2080);

        await_arrival;
        mark("D == one +90 leg done, zero already moved");
        dpos=2;ms;check(!reached);
        dpos=0;repeat(199)ms;check(!reached);
        ms;check(reached);
        mark("E == settled again after zero move");

        keys(0,0,1);check(target_cmd==17039360&&moving&&!reached);
        follow_ramp(17039360,2100);check(n==2080);
        await_arrival;
        mark("F == second +90 leg done");
        keys(1,0,0);check(target_cmd==-17039360);
        follow_ramp(-17039360,2100);check(n==2080);
        mark("G == one -90 leg done");

        keys(1,0,1);check(target_cmd==0);
        keys(0,1,1);check(target_cmd==0);
        follow_ramp(0,2100);check(n==2080);
        await_arrival;

        actual_pos=-200;
        @(negedge clk);re_zero=1;actual_pos=0;
        @(negedge clk);re_zero=0;#1;
        check(target_ref==0&&target_cmd==0&&!ready&&!home_fault);
        check(!key_arrival);
        repeat(300)ms;check(homing&&!ready);
        repeat(200)ms;check(ready&&reached);

        active=0;actual_pos=-40;ms;active=1;ms;
        repeat(300)ms;check(homing&&target_cmd==0&&target_ref>=(-40<<16)&&target_ref<0);
        follow_ramp(0,1400);repeat(200)ms;check(ready);

        active=0;actual_pos=521;ms;active=1;ms;
        check(home_fault&&!ready&&!moving&&target_ref==(521<<16));

        keys(0,0,1);repeat(10)ms;check(home_fault&&target_cmd==(521<<16));
        check(dut.pending_key==2'd0);
        active=0;actual_pos=-521;ms;active=1;ms;check(home_fault);

        @(negedge clk);re_zero=1;actual_pos=0;
        @(negedge clk);re_zero=0;#1;
        repeat(500)ms;check(ready&&!home_fault);

        active=0;actual_pos=100;ms;active=1;ms;repeat(300)ms;
        angle_error=61;previous_ref=target_ref;
        repeat(29998)ms;check(!home_fault&&paused);
        repeat(2)ms;check(home_fault&&!ready&&!moving&&target_ref==previous_ref);

        fault=1;ms;keys(1,0,0);
        check(!ready&&!homing&&!home_fault);
        check(dut.pending_key==2'd0);
        fault=0;active=0;ms;check(!ready);
        $display("ALL PASS key_motion_target checks=%0d",checks);$finish;
    end
endmodule
