`timescale 1ns/1ps
module tb_pend_top;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,tick=0,key=1;
    reg key_neg_n=1,key_zero_n=1,key_pos_n=1;

    reg [52*32-1:0] params=0;
    reg [15:0] theta=3095;
    reg signed [31:0] encoder=10000;
    wire pwm,an1,an2,pwm_b,bn1,bn2,adc_clk,adc_oe,tx;
    wire [3:0] led;
    integer k,checks=0;
    reg [63:0] expected_probe;
    reg expected_tick;
    reg signed [31:0] motion_ref_before;
    always @(posedge clk) if(rst_n) begin
        expected_probe={dut.ch_duty,dut.ch_pos,dut.ch_delta,dut.ch_theta};
        expected_tick=tick;
        #1;
        if(dut.probe_data!==expected_probe || dut.probe_tick!==expected_tick)
            $fatal(1,"probe/strobe pipeline alignment");
    end
    pend_top dut(.sw1_n(key),.sw2_n(key_neg_n),.sw3_n(key_zero_n),.sw4_n(key_pos_n),
        .clk(clk),.rst_n(rst_n),.uart_rx_pin(1'b1),.uart_tx_pin(tx),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe),.adc_data_in(10'd740),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.pwm(pwm),.an1(an1),.an2(an2),
        .pwm_b(pwm_b),.bn1(bn1),.bn2(bn2),.led(led));
    task step;begin
        @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(12)@(negedge clk);
    end endtask
    task check;input ok;begin checks=checks+1;if(ok!==1)
        $fatal(1,"top %0d origin=%0d pos=%0d state=%0d mark=%b long=%b allowed=%b hold=%0d safe=%b posfault=%b",
            checks,dut.u_position.origin,dut.pos_off,dut.swing_state,dut.zero_mark_pulse,
            dut.zero_long_press,dut.zero_mark_allowed,dut.u_sw3_button.hold_count,
            dut.safe,dut.position_fault);end endtask
    initial begin

        check(dut.INNER_DIV==1);
        check(dut.OUTER_DIV==25);
        check(dut.INNER_DIV*dut.OUTER_DIV==25);

        check(dut.u_dbg.u_proto.u_params.MAGIC==32'h0002_0719);
        force dut.param_flat=params;
        force dut.ctrl_tick=tick;
        force dut.theta=theta;
        force dut.enc_pos=encoder;
        force dut.enc_delta=16'sd0;
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

        #1;
        check(dut.u_pid_ang.en_dlpf==1 && dut.u_pid_ang.en_dspan2==1);
        check(dut.u_pid_pos.en_dlpf==1 && dut.u_pid_pos.en_dspan2==0);

        check(dut.u_pid_ang.en_dspan4==0 && dut.u_pid_pos.en_dspan4==0);
        check(dut.u_pid_ang.lpf_shift==2'd0 && dut.u_pid_pos.lpf_shift==2'd0);
        params[9*32+:32]=7|16|128;
        #1;
        check(dut.u_pid_pos.en_dspan4==1 && dut.u_pid_pos.lpf_shift==2'd2);
        check(dut.u_pid_ang.en_dspan4==0 && dut.u_pid_ang.lpf_shift==2'd0);
        params[9*32+:32]=271;
        #1;
        check(dut.u_pid_ang.fuzzy_enable && dut.u_pid_pos.fuzzy_enable);
        params[9*32+:32]=7;
        #1;
        check(!dut.u_pid_ang.fuzzy_enable && !dut.u_pid_pos.fuzzy_enable);
        repeat(4) @(negedge clk); rst_n=1;
        params[33*32+:32]=-(2<<16); params[34*32+:32]=-(30<<16);
        params[35*32+:32]=-(3<<16); params[36*32+:32]=-(36<<16);
        params[37*32+:32]=100; params[38*32+:32]=200;
        force dut.phase_ms=16'd10;
        repeat(3) @(negedge clk);
        check(dut.u_pid_pos.kp==-(2<<16) && dut.u_pid_pos.kd==-(30<<16));
        force dut.phase_ms=16'd150;
        repeat(3) @(negedge clk);
        check(dut.u_pid_pos.kp==-(3<<16) && dut.u_pid_pos.kd==-(36<<16));
        force dut.phase_ms=16'd301;
        repeat(3) @(negedge clk);
        check(dut.u_pid_pos.kp==-78643 && dut.u_pid_pos.kd==-(24<<16));
        release dut.phase_ms;
        rst_n=0;
        params[37*32+:32]=0; params[38*32+:32]=0;
        params[9*32+:32]=7;
        #1;
        repeat(4)@(negedge clk);rst_n=1;step;
        check(dut.duty_cmd==0 && !pwm && !pwm_b);
        params[16*32+:32]=2;step;
        params[13*32+:32]=26<<16;#1;
        check(dut.position_target==32'sd1703936);
        params[13*32+:32]=0;#1;
        encoder=10004;step;
        check(dut.pos_off==4 && !dut.position_fault);

        encoder=10000+40*1040;repeat(48)@(negedge clk);
        check(!dut.position_fault && dut.pos_off==0);
        encoder=10000;repeat(48)@(negedge clk);
        check(!dut.position_fault && dut.pos_off==0 && dut.u_position.origin==10000);

        theta=1017;params[16*32+:32]=3;
        for(k=0;k<140;k=k+1)step;
        check(dut.swing_state==2 && dut.duty_cmd<0 && dut.duty_cmd>=-600 && !dut.mode_close);
        check(dut.ch_pos==0);

        encoder=10042;step;check(dut.ch_pos==42);
        params[8*32+:32]=450;
        theta=2017;step;theta=2900;repeat(25)step;
        theta=2930;step;
        check(dut.swing_state==6 && dut.mode_close && !dut.pid_clear && dut.swing_entered);
        params[13*32+:32]=26<<16;#1;

        check(!dut.motion_active && dut.position_target==32'sd0);
        repeat(6)step;
        check(dut.ang_out!=0 && dut.duty_cmd!=0 && !dut.swing_confirmed);
        theta=3095;
        for(k=0;k<35;k=k+1)step;
        check(dut.auto_balance && dut.mode_close && dut.pos_off==42 && dut.motion_active);
        check(dut.position_target==dut.motion_target_ref && dut.position_target==(42<<16) && !dut.motion_ready);
        for(k=0;k<310;k=k+1)step;
        check(dut.motion_homing && !dut.motion_ready && dut.motion_target_cmd==0);
        for(k=0;k<1600;k=k+1)begin
            encoder=10000+$signed(dut.motion_target_ref[31:16]);step;
        end
        check(dut.motion_ready && dut.motion_active && dut.position_target==0);
        check(dut.u_dbg.control_status[21:18]==4'b1001);
        key_pos_n=0;repeat(21)step;
        check(dut.motion_target_cmd==0);
        key_pos_n=1;repeat(21)step;
        check(dut.motion_target_cmd==32'sd17039360);
        step;
        check(dut.motion_target_ref>0 && dut.position_target==dut.motion_target_ref);
        check(dut.u_dbg.control_status[21:18]==4'b1100);
        check(dut.motion_traveling && !dut.motion_recovering);
        check(dut.u_pid_pos.kp==-157286 && dut.u_pid_pos.ki==0);
        check(dut.u_pid_ang.target==(3095<<16) && dut.u_pid_ang.kp==(3<<16));

        motion_ref_before=dut.motion_target_ref;theta=3335;step;

        check(dut.motion_recovering && dut.motion_paused &&
              dut.motion_target_ref==motion_ref_before && !dut.safe);
        check(dut.u_pid_pos.kp==-78643 && dut.u_pid_pos.ki==0);

        check(dut.outer_src==($signed(dut.outer_out)>>>2));
        check(dut.u_pid_ang.kd==(30<<16) && dut.u_pid_pos.kd==-(24<<16));
        theta=3295;repeat(3)step;check(dut.motion_recovering);
        theta=3095;repeat(3)step;
        check(!dut.motion_recovering && dut.u_pid_pos.kp==-157286);
        check(dut.outer_src===dut.outer_out);

        force dut.kp_arm_sel=32'h80000000;repeat(3)@(negedge clk);
        check(dut.kp_arm_control==32'sh80000000);
        force dut.kp_arm_sel=32'h7fffffff;repeat(3)@(negedge clk);
        check(dut.kp_arm_control==32'sh7fffffff);
        release dut.kp_arm_sel;repeat(3)@(negedge clk);
        key_pos_n=1;repeat(21)step;
        key_zero_n=0;repeat(21)step;
        check(dut.motion_target_cmd==32'sd17039360);
        key_zero_n=1;repeat(21)step;
        check(dut.motion_target_cmd==0);
        key_neg_n=0;repeat(21)step;
        check(dut.motion_target_cmd==0);
        key_neg_n=1;repeat(21)step;
        check(dut.motion_target_cmd==-32'sd17039360);
        motion_ref_before=dut.motion_target_ref;
        step;
        check(dut.motion_target_ref<motion_ref_before);
        key_neg_n=1;repeat(21)step;

        encoder=10030;repeat(3)step;
        params[9*32+:32]=271;
        check(dut.u_position.origin==10000 && dut.pos_off==30);
        key_zero_n=0;repeat(2019)step;
        check(dut.u_position.origin==10000);
        repeat(2)step;repeat(4)@(negedge clk);
        check(dut.u_position.origin==10030 && dut.pos_off==0 && dut.ch_pos==0);
        check(dut.motion_target_ref==0 && dut.motion_target_cmd==0 && !dut.motion_ready);
        check(dut.u_dbg.control_status[22] && dut.position_pid_clear && !dut.pid_clear);
        check(dut.outer_out==0 && dut.u_pid_pos.err_prev==0 && dut.u_pid_pos.u_pid.err_d==0);
        repeat(40)@(negedge clk);check(!dut.position_pid_clear);
        repeat(2050)step;check(dut.u_position.origin==10030);
        key_zero_n=1;repeat(21)step;
        check(dut.motion_ready && dut.motion_target_cmd==0);

        key_neg_n=0;repeat(2019)step;
        check(!dut.profile_busy && dut.motion_target_cmd==0);
        repeat(2)step;
        check(dut.profile_busy && !dut.profile_id && dut.profile_segment==0);
        check(dut.motion_target_cmd==-(130<<16) && dut.u_key_motion.current_slew==1136);
        check(dut.u_dbg.control_status[25] && !dut.u_dbg.control_status[28]);
        key_neg_n=1;repeat(21)step;check(dut.profile_busy);
        key_zero_n=0;repeat(21)step;key_zero_n=1;repeat(21)step;
        check(!dut.profile_busy && dut.motion_target_cmd==0);
        key_pos_n=0;repeat(2021)step;
        check(dut.profile_busy && dut.profile_id && dut.motion_target_cmd==(260<<16));
        check(dut.u_key_motion.current_slew==1515 && dut.u_dbg.control_status[28]);
        key_pos_n=1;repeat(21)step;
        key_zero_n=0;repeat(2021)step;
        check(!dut.profile_busy && !dut.motion_ready && dut.motion_target_cmd==0);
        key_zero_n=1;repeat(550)step;
        check(dut.motion_ready && dut.motion_target_cmd==0);
        theta=4000;step;step;
        check(dut.swing_state==5 && dut.duty_cmd==0 && !dut.motion_ready);
        params[16*32+:32]=0;step;check(dut.duty_cmd==0 && !pwm_b);
        check(dut.u_position.origin==10030);
        encoder=10070;repeat(3)step;
        key_zero_n=0;repeat(2021)step;key_zero_n=1;repeat(21)step;
        check(dut.u_position.origin==10070 && dut.ch_pos==0 && !dut.motion_ready);

        release dut.param_flat;
        encoder=10000;rst_n=0;step;rst_n=1;theta=1017;
        repeat(25)step;check(dut.param_flat[16*32+:32]==0);
        key=0;repeat(21)step;check(dut.param_flat[16*32+:32]==3);

        repeat(140)step;check(dut.swing_state==2 && dut.duty_cmd<0 && dut.duty_cmd>=-900);
        theta=2017;step;theta=3017;step;theta=3095;repeat(35)step;
        check(dut.auto_balance && dut.mode_close && dut.pos_off==0);
        key=1;repeat(21)step;key=0;repeat(21)step;
        check(dut.param_flat[16*32+:32]==0 && dut.duty_cmd==0 && !pwm_b);

        key=1;repeat(21)step;key=0;repeat(21)step;
        check(dut.param_flat[16*32+:32]==3);
        encoder=49556; theta=1017;
        repeat(140)step;check(dut.swing_state==2);
        theta=2017;step;theta=3017;step;theta=3095;
        for(k=0;k<300 && !dut.motion_active;k=k+1)step;
        check(dut.motion_active && dut.auto_balance && dut.mode_close);
        check(dut.pos_off==0);
        encoder=49706;step;check(dut.pos_off==150);
        key=1;repeat(21)step;key=0;repeat(21)step;
        check(dut.param_flat[16*32+:32]==0 && dut.duty_cmd==0 && !pwm_b);
        $display("ALL PASS top %0d",checks);$finish;
    end
endmodule
