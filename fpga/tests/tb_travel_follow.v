`timescale 1ns/1ps

module tb_travel_follow;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,tick=0,active=0,balance_confirmed=0,re_zero=0,fault=0;
    reg key_neg=0,key_zero=0,key_pos=0;
    reg signed [15:0] angle_error=0,actual_pos=0,dpos=0;
    reg signed [31:0] travel_follow_sel=0;
    wire signed [31:0] target_ref,target_cmd;
    wire ready,moving,paused,reached,homing,home_fault,traveling,recovering;
    integer checks=0,n,lead,max_lead,loop_gap;
    reg signed [31:0] previous_ref,freeze_a;

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

        .travel_follow_sel(travel_follow_sel),.home_leg(),
        .target_ref(target_ref),.target_cmd(target_cmd),.ready(ready),
        .moving(moving),.paused(paused),.reached(reached),
        .homing(homing),.home_fault(home_fault),.at_target(),
        .traveling(traveling),.recovering(recovering));

    task check; input ok;
        begin checks=checks+1;
            if(ok!==1) $fatal(1,"follow %0d ref=%0d cmd=%0d pos=%0d lead=%0d ready=%b",
                checks,target_ref,target_cmd,actual_pos,max_lead,ready);
        end
    endtask
    task ms; begin
        @(negedge clk); tick=1; @(negedge clk); tick=0; #1;
    end endtask
    task keys; input neg,zero,pos; begin
        @(negedge clk); key_neg=neg;key_zero=zero;key_pos=pos;
        @(negedge clk);key_neg=0;key_zero=0;key_pos=0;#1;
    end endtask

    task ms_lead; begin
        ms;
        lead = target_ref[31:16]-actual_pos;
        if(lead>max_lead) max_lead=lead;
    end endtask

    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        actual_pos=0;dpos=0;ms;
        active=1;balance_confirmed=1;

        repeat(600)ms;check(ready&&!homing&&!home_fault);

        keys(0,0,1);
        check(target_cmd==(260<<16)&&moving&&!reached&&!homing);
        max_lead=0;
        for(n=0;n<1000;n=n+1) ms_lead;
        freeze_a = target_ref;
        for(n=0;n<1000;n=n+1) ms_lead;
        check(max_lead>16);
        check(max_lead<=97);
        check(target_ref==freeze_a);

        check(target_ref>=(96<<16)&&target_ref<=(97<<16));
        check(dut.travel_phase);
        check(target_cmd==(260<<16));
        check(traveling&&!recovering);

        actual_pos=40;repeat(10)ms;
        check(target_ref>freeze_a&&dut.travel_phase);

        loop_gap = target_cmd - target_ref;
        n=0;
        while(target_ref!=target_cmd&&n<3000) begin
            previous_ref=target_ref;actual_pos=target_ref[31:16];ms;
            check(target_ref-previous_ref<=8192);
            check(target_ref>=previous_ref);
            n=n+1;
        end
        check(target_ref==(260<<16));
        check(n==(loop_gap+8191)/8192);
        check(dut.travel_phase);
        repeat(39)ms;check(dut.travel_phase);
        repeat(2)ms;check(!dut.travel_phase);

        for(n=0;n<1000&&!ready;n=n+1) ms;
        check(ready);
        actual_pos = 0;
        travel_follow_sel = 32'sd64;
        keys(0,0,1);
        max_lead=0;

        for(n=0;n<1000;n=n+1) ms_lead;
        check(max_lead>16);
        check(max_lead<=65);
        check(max_lead>=64);
        freeze_a = target_ref;
        for(n=0;n<200;n=n+1) ms_lead;
        check(target_ref==freeze_a);
        travel_follow_sel = 32'sd0;
        for(n=0;n<500;n=n+1) ms_lead;
        check(max_lead>=96&&max_lead<=97);
        check(target_ref>freeze_a);

        n=0;
        while(target_ref!=target_cmd&&n<3000) begin
            actual_pos=target_ref[31:16];ms;n=n+1;
        end
        check(target_ref==target_cmd);

        travel_follow_sel = 32'sd5000;

        for(n=0;n<2000&&(target_ref!=0||dut.travel_phase);n=n+1) ms;
        check(target_ref==0&&!dut.travel_phase);
        check(ready);
        actual_pos = 0;
        keys(0,0,1);
        max_lead=0;
        for(n=0;n<1000;n=n+1) ms_lead;
        check(max_lead>=96&&max_lead<=97);
        travel_follow_sel = 32'sd0;

        $display("ALL PASS travel_follow checks=%0d",checks);$finish;
    end
endmodule
