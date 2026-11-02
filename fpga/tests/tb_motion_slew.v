`timescale 1ns/1ps

module tb_motion_slew;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,tick=0,active=0,balance_confirmed=0,re_zero=0,fault=0;
    reg key_neg=0,key_zero=0,key_pos=0;
    reg signed [15:0] angle_error=0,actual_pos=0,dpos=0;
    wire signed [31:0] target_ref,target_cmd;
    wire ready,moving,paused,reached,homing,home_fault;
    integer checks=0,n,last_step,sec=0;
    reg signed [31:0] previous_ref;

    reg signed [31:0] slew_sel=0, prof_slew_sel=0, accel_sel=0, decel_sel=0;
    reg profile_request=0;
    reg signed [31:0] profile_target=0, profile_slew=0;

    key_motion_target dut(
        .clk(clk),.rst_n(rst_n),.tick(tick),.active(active),
        .balance_confirmed(balance_confirmed),.re_zero(re_zero),.fault(fault),
        .key_neg(key_neg),.key_zero(key_zero),.key_pos(key_pos),
        .remote_move_req(1'b0),.remote_move_counts(16'sd0),
        .profile_request(profile_request),.profile_target(profile_target),
        .profile_slew(profile_slew),.hold_motion(1'b0),
        .angle_error(angle_error),.actual_pos(actual_pos),.dpos(dpos),
        .slew_sel(slew_sel),.prof_slew_sel(prof_slew_sel),.accel_sel(accel_sel),

        .decel_sel(decel_sel),

        .home_slew_sel(32'sd0),
        .target_ref(target_ref),.target_cmd(target_cmd),.ready(ready),
        .moving(moving),.paused(paused),.reached(reached),
        .homing(homing),.home_fault(home_fault),.at_target(),
        .traveling(),.recovering(),.home_leg());

    task check; input ok;
        begin checks=checks+1;
            if(ok!==1) $fatal(1,"slew sec=%0d chk=%0d ref=%0d cmd=%0d ready=%b homing=%b fault=%b last=%0d slew=%0d acc=%0d dec=%0d",
                sec,checks,target_ref,target_cmd,ready,homing,home_fault,last_step,slew_sel,accel_sel,decel_sel);
        end
    endtask
    task ms; begin
        @(negedge clk); tick=1; @(negedge clk); tick=0; #1;
    end endtask
    task keys; input neg,zero,pos; begin
        @(negedge clk); key_neg=neg;key_zero=zero;key_pos=pos;
        @(negedge clk);key_neg=0;key_zero=0;key_pos=0;

        repeat(3)@(negedge clk);#1;
    end endtask

    task follow_ramp_lim; input signed [31:0] goal; input integer timeout; input integer maxstep;
        integer count;
        begin
            count=0; last_step=0;
            while(target_ref!=goal) begin
                previous_ref=target_ref; actual_pos=target_ref[31:16];ms;
                last_step=target_ref-previous_ref;
                check((last_step<=maxstep)&&(last_step>=-maxstep));
                if(goal>previous_ref) check(target_ref>=previous_ref && target_ref<=goal);
                else check(target_ref<=previous_ref && target_ref>=goal);
                count=count+1;
                if(count>timeout)$fatal(1,"slew: ramp timeout");
            end
            actual_pos=goal[31:16];n=count;
        end
    endtask

    initial begin

        repeat(3)@(negedge clk);rst_n=1;
        actual_pos=80;ms;check(target_ref==(80<<16) && !ready);
        keys(0,0,1);check(target_cmd==(80<<16));
        active=1;ms;check(target_ref==(80<<16)&&!home_fault);
        repeat(25)ms;keys(1,0,0);check(target_cmd==(80<<16)&&!ready&&!homing);
        balance_confirmed=1;
        angle_error=61;repeat(10)ms;check(!dut.qualified);
        angle_error=-32768;ms;angle_error=60;dpos=2;ms;dpos=-32768;ms;dpos=1;
        repeat(299)ms;check(!dut.qualified&&!ready);
        @(negedge clk);tick=1;key_pos=1;
        @(negedge clk);tick=0;key_pos=0;#1;
        check(homing && moving && !ready && target_cmd==0 && target_ref==(80<<16));
        previous_ref=target_ref;angle_error=61;ms;
        check(paused&&target_ref==previous_ref);
        angle_error=-60;actual_pos=63;ms;check(paused&&target_ref==previous_ref);
        actual_pos=64;ms;check(!paused&&target_ref<previous_ref);
        keys(0,0,1);check(target_cmd==0);
        angle_error=0;dpos=0;
        follow_ramp_lim(0,2600,8192);check(target_ref==0&&!ready&&!moving);
        repeat(199)ms;check(!ready);
        ms;check(ready&&reached&&!homing);

        keys(0,0,1);check(target_cmd==17039360&&moving&&!reached);
        follow_ramp_lim(17039360,2100,8192);check(n==2080);

        sec=1;
        slew_sel=32'sd0;
        keys(1,0,0); check(target_cmd==-17039360);
        follow_ramp_lim(-17039360,4200,8192); check(n==4160);

        sec=2;
        slew_sel=32'sd16384;
        keys(0,0,1); check(target_cmd==17039360);
        follow_ramp_lim(17039360,4200,16384); check(n==2080);

        sec=3;
        slew_sel=32'sd32768;
        keys(1,0,0); check(target_cmd==-17039360);
        follow_ramp_lim(-17039360,4200,32768); check(n==1040);

        sec=4;
        slew_sel=-32'sd1;
        keys(0,0,1); check(target_cmd==17039360);
        follow_ramp_lim(17039360,4200,8192); check(n==4160);

        sec=5;
        slew_sel=32'sd0; accel_sel=32'sd4096;
        keys(1,0,0); check(target_cmd==-17039360);

        actual_pos=target_ref[31:16];previous_ref=target_ref;ms;
        check(target_ref-previous_ref==-2048);

        actual_pos=target_ref[31:16];previous_ref=target_ref;ms;
        check(target_ref-previous_ref==-6144);

        actual_pos=target_ref[31:16];previous_ref=target_ref;ms;
        check(target_ref-previous_ref==-8192);

        follow_ramp_lim(-17039360,6000,8192);
        check(target_ref==-17039360);

        check(last_step<=2048);

        sec=6;
        accel_sel=32'sd0;
        keys(0,0,1); check(target_cmd==17039360);
        follow_ramp_lim(17039360,4200,8192); check(n==4160);
        check(last_step==8192);

        sec=7;
        accel_sel=32'sd4096;
        keys(1,0,0); check(target_cmd==-17039360);
        follow_ramp_lim(-17039360,6000,8192);
        check(n>4160);

        sec=8;
        accel_sel=32'sd0; decel_sel=32'sd4096; slew_sel=32'sd0;
        keys(0,0,1); check(target_cmd==17039360);

        check(dut.use_exact_brake==1'b1);
        check(dut.stop_done==1'b1);
        follow_ramp_lim(17039360,7000,8192);
        check(target_ref==17039360);
        check(last_step<=2048);
        check(n>4160);

        sec=9;
        decel_sel=-32'sd1;
        keys(1,0,0); check(target_cmd==-17039360);
        follow_ramp_lim(-17039360,4200,8192); check(n==4160);

        check(last_step==-8192);

        sec=10;
        accel_sel=32'sd4096; decel_sel=32'sd8192;
        keys(0,0,1); check(target_cmd==17039360);
        actual_pos=target_ref[31:16];previous_ref=target_ref;ms;
        check(target_ref-previous_ref==2048);
        actual_pos=target_ref[31:16];previous_ref=target_ref;ms;
        check(target_ref-previous_ref==6144);
        actual_pos=target_ref[31:16];previous_ref=target_ref;ms;
        check(target_ref-previous_ref==8192);
        follow_ramp_lim(17039360,7000,8192);
        check(last_step<=2048);
        check(n>4160);

        accel_sel=32'sd0; slew_sel=32'sd0; decel_sel=32'sd0;

        profile_target=32'sd0; profile_slew=32'sd1000000; prof_slew_sel=32'sd2100;
        @(negedge clk);profile_request=1;
        @(negedge clk);profile_request=0;#1;
        check(dut.current_slew==32'sd2100);

        prof_slew_sel=32'sd0; profile_slew=32'sd1000000;
        @(negedge clk);profile_request=1;
        @(negedge clk);profile_request=0;#1;
        check(dut.current_slew==32'sd3787);

        prof_slew_sel=32'sd2100; profile_slew=32'sd1000;
        @(negedge clk);profile_request=1;
        @(negedge clk);profile_request=0;#1;
        check(dut.current_slew==32'sd1000);

        prof_slew_sel=-32'sd5; profile_slew=32'sd1000000;
        @(negedge clk);profile_request=1;
        @(negedge clk);profile_request=0;#1;
        check(dut.current_slew==32'sd3787);

        $display("ALL PASS motion_slew checks=%0d",checks);$finish;
    end
endmodule
