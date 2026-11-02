`timescale 1ns/1ps
module tb_curve_endpoint #(parameter integer ENDPOINT=10260, MANUAL_STOP=0);
    wire pwm,an1,an2,pwm_b,bn1,bn2,adc_clk,adc_oe,tx;
    wire [3:0] led;

    localparam [31:0] SENT = 32'sd777 <<< 16;

    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,tick=0,key=1,seen_outer=0;

    reg [52*32-1:0] params=0;
    reg [15:0] theta=3095;
    reg signed [31:0] encoder=10000;
    reg signed [15:0] delta=0;
    integer checks=0, i, r0;

    pend_top dut(.sw1_n(key),.sw2_n(1'b1),.sw3_n(1'b1),.sw4_n(1'b1),
        .clk(clk),.rst_n(rst_n),.uart_rx_pin(1'b1),.uart_tx_pin(tx),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe),.adc_data_in(10'd740),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.pwm(pwm),.an1(an1),.an2(an2),
        .pwm_b(pwm_b),.bn1(bn1),.bn2(bn2),.led(led));

    task check; input ok; input [8*64-1:0] name; begin
        checks=checks+1;
        if(ok!==1) $fatal(1,"spin [%0s] chk=%0d ref=%0d latch=%b ang=%0d vel=%0d act=%b",
            name,checks,$signed(dut.spin_ref),dut.spin_tilt_latch,
            $signed(dut.spin_angle_abs),$signed(dut.vel_sum),dut.spin_active);
    end endtask

    task ct; begin
        @(negedge clk); tick=1;
        @(posedge clk); #1;
        if (dut.outer_tick===1'b1) seen_outer=1'b1;
        @(negedge clk); tick=0;
        repeat(40)@(negedge clk);
    end endtask

    task outers; input integer n; integer j; begin
        for(j=0;j<25*n;j=j+1) ct;
    end endtask

    task align_outer; integer j; begin
        seen_outer=1'b0; j=0;
        while(!seen_outer && j<60) begin ct; j=j+1; end
        if(!seen_outer) $fatal(1,"no outer tick within 60 ctrl ticks");
    end endtask

    reg bus_en=0;reg[7:0]bus_addr=0;reg[31:0]bus_data=0;
    task wr;input[7:0]a;input[31:0]v;begin
      @(negedge clk);bus_addr=a;bus_data=v;bus_en=1;
      @(negedge clk);bus_en=0;repeat(5)@(negedge clk);
    end endtask
    initial begin
        check(dut.OUTER_DIV==25,"outer period is 25ms");

        check(dut.u_dbg.u_proto.u_params.MAGIC==32'h0002_0719,"0x0207 identity");
        force dut.param_flat=params;
        force dut.ctrl_tick=tick;
        force dut.theta=theta;
        force dut.enc_pos=encoder;
        force dut.enc_delta=delta;
        force dut.adc_valid=1'b1;
        force dut.theta_raw=12'd2960;
        force dut.enc_fault_seen=1'b0;

        params[3*32+:32]=3<<16;params[4*32+:32]=30<<16;
        params[5*32+:32]=-78643;params[20*32+:32]=-(24<<16);
        params[11*32+:32]=1024;params[19*32+:32]=2;params[18*32+:32]=3095;
        params[7*32+:32]=15;params[8*32+:32]=120;
        params[12*32+:32]=4<<16;params[14*32+:32]=104858;
        params[24*32+:32]=-600;params[25*32+:32]=600;params[26*32+:32]=1000;
        params[29*32+:32]=341;params[30*32+:32]=10;
        params[27*32+:32]=450;params[28*32+:32]=1017;
        params[9*32+:32]=7;

        params[40*32+:32]=32'h0000_2000;params[41*32+:32]=32'd3787;params[42*32+:32]=32'd0;
        params[43*32+:32]=32'd10;params[44*32+:32]=-(8<<16);params[45*32+:32]=32'd0;
        params[46*32+:32]=32'd4;
        #1;
        repeat(4)@(negedge clk);rst_n=1;ct;ct;

        params[16*32+:32]=3;force dut.u_swing.state=3'd4;
        force dut.motion_ready=0;force dut.motion_at_target=0;force dut.profile_busy=1;
        force dut.vel_sum=4;
        force dut.curve_wr_en=bus_en;force dut.curve_wr_addr=bus_addr;force dut.curve_wr_data=bus_data;
        theta=3185;ct;ct;
        check(dut.curve_start_ready,"relaxed start ignores old key readiness/profile busy");
        dut.u_key_motion.pending_key=2;wr(64,3);
        check(dut.curve_owned&&dut.u_curve.state==4,"trajectory claims active key motion");
        release dut.profile_busy;repeat(5)@(negedge clk);
        check(dut.u_key_motion.pending_key==0&&!dut.profile_command_valid&&dut.u_profile.state==0,"claim flushes pending keys and profile sequencer");
        encoder=ENDPOINT;repeat(8)@(negedge clk);
        theta=3225;ct;ct;check(!dut.curve_start_ready,"angle still gates actual launch");
        wr(64,0);wr(67,32'h20702);wr(65,10);wr(66,10);wr(68,0);wr(64,1);
        repeat(10)ct;check(dut.u_curve.state==4&&dut.u_curve.elapsed==0,"uploaded curve waits without consuming time");
        theta=3185;ct;ct;check(dut.u_curve.state==1,"recovery automatically launches pending curve");
        if(MANUAL_STOP) wr(64,2);
        theta=3095;force dut.vel_sum=0;encoder=ENDPOINT;outers(11);
        check(dut.curve_holding&&dut.u_position.origin==ENDPOINT&&dut.pos_off==0,"curve settling moves origin to actual endpoint");
        check(dut.curve_hold_actual==0&&!dut.curve_owned,"settled curve hands off at the new endpoint");
        check(dut.motion_target_ref==0 && dut.motion_target_cmd==0 && dut.position_target_r==0,
              "curve endpoint must rebase key references atomically");
        check(dut.position_error==0 && dut.outer_out==0,
              "stationary endpoint must not create a position-PID kick");
        encoder=(ENDPOINT+1);ct;check(dut.u_position.origin==ENDPOINT&&dut.pos_off==1,"origin only changes on arrival pulse");

        release dut.motion_ready;release dut.motion_at_target;release dut.vel_sum;
        ct;ct;
        r0=dut.u_position.origin;
        for(i=1;i<=2;i=i+1) begin
          @(negedge clk);
          dut.u_key_motion.qualified=1;dut.u_key_motion.ready=1;dut.u_key_motion.was_active=1;
          dut.u_key_motion.target_ref=260*65536;dut.u_key_motion.target_cmd=260*65536;
          dut.u_key_motion.travel_phase=1;dut.u_key_motion.key_leg=1;dut.u_key_motion.home_leg=0;
          encoder=r0+i*260;repeat(8)@(negedge clk);repeat(40)ct;
          check(dut.u_position.origin==encoder&&dut.pos_off==0,"each key leg establishes a new zero");
          check(dut.motion_target_ref==0&&dut.motion_target_cmd==0,"key target and actual share rebased coordinates");
        end
        encoder=encoder+5;repeat(8)@(negedge clk);
        force dut.profile_done=1'b1;repeat(5)@(negedge clk);
        check(dut.u_position.origin==encoder&&dut.motion_target_ref==0,"whole profile completion rebases target and origin");
        release dut.profile_done;
        $display("ALL PASS curve endpoint %0d",checks);$finish;
    end
    always @(posedge clk) if(rst_n && dut.curve_settled_entry) begin
        #1;
        if(dut.pos_off!==0 || dut.motion_target_ref!==0 || dut.motion_target_cmd!==0 || dut.position_target_r!==0)
            $fatal(1,"actual and reference must change on the SAME clock");
    end
    initial begin #10000000;$fatal(1,"timeout");end
endmodule
