`timescale 1ns/1ps

module tb_motion_coordination;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,tick=0,neg=0,pos=0;
    reg [52*32-1:0] params=0;
    reg signed [31:0] encoder=10000;
    reg signed [15:0] delta=0;
    integer checks=0,i,direction,only_handoff,unused;
    pend_top dut(.clk(clk),.rst_n(rst_n),.sw1_n(1'b1),.sw2_n(1'b1),
        .sw3_n(1'b1),.sw4_n(1'b1),.uart_rx_pin(1'b1),.adc_data_in(10'd740),
        .adc_otr(1'b0),.enc1_a(1'b0),.enc1_b(1'b0));
    task check;input ok;input[8*100-1:0] label;begin
        checks=checks+1;
        if(ok!==1)$fatal(1,"coordination %0d: %0s ref=%0d D=%0d origin=%0d",
            checks,label,$signed(dut.motion_target_ref),
            $signed(dut.u_pid_pos.u_pid.dd_used),$signed(dut.u_position.origin));
    end endtask
    task ms;begin
        @(negedge clk);tick=1;@(negedge clk);tick=0;
        repeat(40)@(negedge clk);
    end endtask
    task begin_run;begin
        rst_n=0;encoder=10000;delta=0;neg=0;pos=0;params[16*32+:32]=3;
        repeat(4)@(negedge clk);rst_n=1;repeat(600)ms;
        check(dut.motion_ready,"ready before a move");
    end endtask
    task press;input integer sign;begin
        @(negedge clk);neg=sign<0;pos=sign>0;
        @(negedge clk);neg=0;pos=0;
    end endtask
    initial begin
        only_handoff=0;unused=$value$plusargs("only_handoff=%d",only_handoff);
        repeat(4)@(negedge clk);params=dut.param_flat;
        params[18*32+:32]=3095;
        force dut.param_flat=params;force dut.ctrl_tick=tick;
        force dut.theta=16'd3095;force dut.theta_raw=12'd2960;
        force dut.enc_pos=encoder;force dut.enc_delta=delta;
        force dut.adc_valid=1'b1;force dut.enc_fault_seen=1'b0;
        force dut.u_swing.state=3'd4;
        force dut.remote_key_neg=neg;force dut.remote_key_pos=pos;
        if(!only_handoff)begin
            for(direction=-1;direction<=1;direction=direction+2)begin
                begin_run;press(direction);repeat(200)ms;
                check(dut.motion_traveling && dut.u_pid_pos.u_pid.pp_s!=0,"position P still tracks reference");

                check(dut.u_pid_pos.u_pid.dd_used==0,"stationary arm: moving reference must not inject D drive");
                encoder=encoder+3*direction;delta=direction;ms;delta=0;repeat(24)ms;
                check(direction*$signed(dut.u_pid_pos.u_pid.dd_used)>0,"D still damps actual arm motion");
            end
            begin_run;
            @(negedge clk);dut.u_pid_pos.u_pid.integ=17<<16;
            params[16*32+:32]=2;repeat(25)ms;
            check(dut.u_pid_pos.u_pid.dd_used==0,"auto to manual: seed changed D coordinate without a kick");
            check(dut.u_pid_pos.u_pid.integ==(17<<16),"changing D source must preserve the integral contribution");
            params[13*32+:32]=10<<16;
            repeat(25)ms;
            check(dut.u_pid_pos.u_pid.dd_used!=0,"manual position mode retains its existing derivative response");
            params[13*32+:32]=0;
            begin_run;params[16*32+:32]=2;repeat(25)ms;
            params[16*32+:32]=3;repeat(25)ms;
            check(dut.u_pid_pos.u_pid.dd_used==0,"manual to auto: seed measured-position D history");
        end
        begin_run;press(1);

        @(negedge clk);dut.u_key_motion.target_ref=260<<16;
        dut.u_key_motion.target_cmd=260<<16;encoder=10255;
        for(i=0;i<40;i=i+1)begin
            delta=(i%8==0);if(delta)encoder=encoder+1;
            ms;
            check(dut.u_position.origin==10000 && dut.motion_traveling,
                  "zero/one instantaneous increment must not hand off a moving arm");
        end
        delta=1;encoder=encoder+1;ms;delta=0;
        repeat(39)begin ms;check(dut.u_position.origin==10000,"requires a consecutive stop window");end
        ms;
        check(dut.u_position.origin==encoder && dut.pos_off==0,"settled endpoint becomes the new zero");
        check(dut.motion_target_ref==0 && dut.motion_target_cmd==0,"target and actual coordinates change together");
        check(!dut.motion_traveling && dut.motion_ready,"handoff keeps key readiness");
        repeat(250)ms;check(dut.motion_reached,"normal balance settling still completes");
        $display("ALL PASS motion coordination %0d",checks);$finish;
    end
endmodule
