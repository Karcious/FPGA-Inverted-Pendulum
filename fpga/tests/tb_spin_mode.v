`timescale 1ns/1ps

module tb_spin_mode;
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
        @(negedge clk);
    end endtask

    task outers; input integer n; integer j; begin
        for(j=0;j<25*n;j=j+1) ct;
    end endtask

    task align_outer; integer j; begin
        seen_outer=1'b0; j=0;
        while(!seen_outer && j<60) begin ct; j=j+1; end
        if(!seen_outer) $fatal(1,"no outer tick within 60 ctrl ticks");
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

        force dut.pos_q16=SENT;

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

        params[16*32+:32]=32'd3;ct;ct;
        force dut.u_swing.state=3'd4;
        repeat(3)@(negedge clk);
        check(dut.motion_active,"mode3 balance reaches BALANCE");
        check(!dut.spin_active,"mode3 balance is NOT spin");
        check(dut.outer_actual_sel===SENT,"mode3 outer reads position (sentinel)");
        check(dut.outer_kd_sel!==32'sd0,"mode3 outer keeps its arm Kd");

        force dut.u_swing.state=3'd0;
        params[16*32+:32]=32'd4;ct;ct;
        check(dut.spin_mode && !dut.motion_active,"mode4 but not balancing");
        check(!dut.spin_active,"NOT balancing => velocity loop stays OFF");
        check(dut.outer_actual_sel===SENT,"swing-up still reads position");

        force dut.u_swing.state=3'd4;ct;ct;
        check(dut.motion_active && dut.spin_active,"mem[16]=4 + BALANCE = spin");
        check(dut.outer_actual_sel!==SENT,"outer actual is NOT position (sentinel)");
        check(dut.outer_actual_sel===dut.vel_q16,"outer actual = arm speed");
        check(dut.outer_kp_sel===params[44*32+:32],"outer Kp = mem[44]");
        check(dut.outer_ki_sel===params[45*32+:32],"outer Ki = mem[45]");
        check(dut.outer_kd_sel===32'sd0,"outer Kd forced to 0 (no D on speed)");
        check(dut.outer_fuzzy_sel===1'b0,"fuzzy layer off in spin");

        check(dut.position_zero_hold!==6'd0,"spin entry holds the outer loop off");

        check(!dut.u_key_motion.active,"spin: key legs are switched off");
        force dut.key_zero_press=1'b1;force dut.key_neg_press=1'b1;
        repeat(4)ct;
        check(!dut.u_key_motion.key_leg,"spin: SW2/SW3 cannot start a leg");
        check(!dut.u_key_motion.key_arrival,"spin: no arrival while spinning");
        check(!dut.u_key_motion.ready,"spin: ready drops (GUI greys the buttons)");
        release dut.key_zero_press;release dut.key_neg_press;
        ct;ct;

        delta=16'sd3;
        outers(3);
        check(dut.vel_sum===20'sd75,"25 ticks x 3 codes = 75 codes/25ms");
        check(dut.outer_actual_sel===(32'sd75<<<16),"measured speed reaches the outer loop");

        delta=16'sd200;
        outers(3);
        check(dut.vel_q16===(32'sd1024<<<16),"velocity saturates at +1024 codes/25ms");
        check(dut.outer_actual_sel===(32'sd1024<<<16),"saturated speed reaches the outer loop");
        delta=16'sd0;
        outers(2);

        params[16*32+:32]=32'd3;ct;ct;
        check(dut.spin_ref===32'sd0,"leaving spin zeroes spin_ref");
        check(!dut.spin_active,"leaving spin disengages the velocity loop");
        check(dut.outer_actual_sel===SENT,"leaving spin restores the position loop");
        params[46*32+:32]=32'd4;
        params[43*32+:32]=32'd100;
        params[16*32+:32]=32'd4;ct;ct;
        check(dut.spin_active && dut.spin_cmd===32'sd64,"mem[43]=100 clamps to +64");

        align_outer; r0=$signed(dut.spin_ref);
        outers(1);
        check($signed(dut.spin_ref)===r0+4,"soft-start: exactly one step per outer tick");
        check($signed(dut.spin_ref)!==32'sd64,"soft-start: not a step jump");
        align_outer; r0=$signed(dut.spin_ref);
        outers(1);
        check($signed(dut.spin_ref)===r0+4,"soft-start: second step is the same size");
        outers(40);
        check(dut.spin_ref===32'sd64,"soft-start lands exactly on the clamp");
        check(dut.outer_target_sel===(32'sd64<<<16),"outer target = spin setpoint");
        outers(2);
        check(dut.spin_ref===32'sd64,"soft-start holds at the clamp");

        params[43*32+:32]=-32'sd100;ct;ct;
        check(dut.spin_cmd===-32'sd64,"mem[43]=-100 clamps to -64");
        align_outer; r0=$signed(dut.spin_ref);
        outers(1);
        check($signed(dut.spin_ref)===r0-4,"soft-start steps DOWN by mem[46]");

        params[43*32+:32]=32'd0;outers(20);
        check(dut.spin_ref===32'sd0,"setpoint parked at 0");
        params[43*32+:32]=-32'sd100;outers(20);
        check(dut.spin_ref===-32'sd64,"setpoint parked at -64");
        theta=16'd3095+16'd240;ct;ct;ct;
        check(dut.spin_angle_abs>=16'd240,"tilt is measured from mem[18]");
        align_outer; r0=$signed(dut.spin_ref);
        outers(1);
        check(dut.spin_tilt_latch,"tilt >= 240 pulls the setpoint down");
        check(dut.spin_target===32'sd0,"latched target is 0");

        check($signed(dut.spin_ref)===r0+4,"speed setpoint ramps DOWN, not cut");
        outers(20);
        check(dut.spin_ref===32'sd0,"setpoint reaches 0");

        theta=16'd3095+16'd200;ct;ct;ct;outers(2);
        check(dut.spin_tilt_latch,"hysteresis holds at 200");

        theta=16'd3095+16'd160;ct;ct;ct;outers(2);
        check(!dut.spin_tilt_latch,"releases at 160");
        check(dut.spin_cmd===-32'sd64,"command back to the raw setpoint");
        theta=16'd3095;ct;ct;outers(2);

        encoder=12345;
        params[43*32+:32]=32'd0;ct;ct;
        check(dut.spin_cmd===32'sd0,"mem[43]=0 => no spin request");
        outers(30);
        check(dut.spin_ref===32'sd0,"setpoint decays to 0");
        check(dut.spin_parked,"REV15: stopped after a real spin => parked");
        check(dut.u_position.origin===encoder,"park rebases the position origin (new zero)");
        check(!dut.spin_active,"parked: velocity loop released");
        check(dut.outer_actual_sel===SENT,"parked: outer reads position again");
        check(dut.outer_target_sel===32'sd0,"parked: target is the rebased zero");

        repeat(20)@(negedge clk);
        check(!dut.spin_park_req,"park rebase is a one-clock pulse, not a level");

        params[43*32+:32]=32'd10;ct;ct;
        check(!dut.spin_parked,"a fresh setpoint un-parks");
        check(dut.spin_active,"velocity loop back on");
        outers(5);
        check(dut.spin_ref===32'sd10,"soft-start resumes from the new zero");

        params[43*32+:32]=32'd0;ct;ct;
        outers(6);
        check(dut.spin_ref===32'sd0,"⑦c setpoint parked at 0");
        check(dut.spin_hold,"stop phase engages the curve-style position hold");
        check(dut.outer_target_sel===32'sd0,"⑦c hold target is 0");
        check(dut.outer_actual_sel===dut.spin_hold_q16,"⑦c hold actual is the parked position");
        check(dut.outer_kp_sel===dut.kp_arm_control,"⑦c hold uses the arm position Kp");
        check(dut.outer_ki_sel===dut.ki_arm_control,"⑦c hold keeps the arm Ki");
        check(dut.outer_kd_sel===dut.kd_arm_sel,"⑦c hold uses the arm Kd");
        check(dut.outer_d_measured===1'b1,"⑦c hold damps the measured arm motion");
        outers(20);
        check(dut.spin_parked,"⑦c park completes while in the hold");
        check(!dut.spin_active,"⑦c parked: velocity loop released");
        check(dut.u_position.origin===encoder,"⑦c park rebases the origin");

        params[43*32+:32]=32'd10;outers(20);
        check(dut.spin_ref===32'sd10,"setpoint back up");
        params[16*32+:32]=32'd0;ct;ct;
        check(!dut.spin_mode && !dut.spin_active,"coast leaves spin mode");
        check(dut.spin_ref===32'sd0,"leaving spin zeroes spin_ref");
        check(dut.outer_actual_sel===SENT,"position loop restored");
        check(dut.outer_kp_sel!==params[44*32+:32],"outer Kp back to the arm gain");
        check(dut.outer_target_sel===32'sd0,"outer target back to the manual target");

        params[43*32+:32]=32'd0;ct;ct;
        params[16*32+:32]=32'd4;ct;ct;
        check(dut.spin_active,"mode4 + mem[43]=0 (never spun) stays in speed mode");
        outers(20);
        check(!dut.spin_parked,"never spun => no park (REV14 behavior kept)");
        check(dut.spin_ref===32'sd0,"still no rotation requested");
        params[16*32+:32]=32'd0;ct;ct;

        theta=16'd3095;
        params[16*32+:32]=32'd3;params[43*32+:32]=32'd10;
        params[46*32+:32]=32'd4;ct;ct;ct;
        check(dut.motion_active && !dut.travel_mode,"travel: starts idle in mode 3");
        encoder=32'sd20000; delta=16'sd3;
        force dut.travel_req_r=1'b1; force dut.travel_counts_r=16'sd1618;
        @(negedge clk); @(negedge clk);
        release dut.travel_req_r; release dut.travel_counts_r;
        ct;ct;
        check(dut.travel_mode && !dut.travel_done,"travel armed");
        check(dut.travel_target===32'sd1618,"travel target = 1618 codes (1.56 rev)");
        check(dut.spin_active,"travel borrows the velocity loop");
        check(dut.outer_actual_sel===dut.vel_q16,"travel outer actual = arm speed");

        r0=$signed(dut.travel_progress);
        outers(1);
        check($signed(dut.travel_progress)===r0+75,"travel counts 25 x 3 = 75 codes");

        i=0; while(!dut.travel_finish && i<2000) begin ct; i=i+1; end
        check(dut.travel_finish,"travel reaches the target");
        check($signed(dut.travel_progress)>=32'sd1615,"travel counted >1.5 turns");
        check(dut.spin_cmd_eff===32'sd0,"travel command folds to zero at the target");
        delta=16'sd0;
        i=0; while(!dut.spin_parked && i<400) begin ct; i=i+1; end
        check(dut.spin_parked,"travel parks after stopping");
        check(dut.travel_done && dut.travel_count===8'd1,"travel completion counted once");
        check(dut.u_position.origin===encoder,"travel park rebases the origin");
        check(!dut.spin_active,"travel done: velocity loop released");
        check(dut.outer_actual_sel===SENT,"travel done: position loop restored");
        check(dut.outer_target_sel===32'sd0,"travel done: target is the new zero");

        ct;ct;
        check(dut.curve_readouts[352]===1'b1,"addr75 done bit");
        check(dut.curve_readouts[353]===1'b0,"addr75 not aborted");
        check(dut.curve_readouts[352+8 +: 8]===8'd1,"addr75 travel count = 1");
        check(dut.curve_readouts[352+16 +: 16]===dut.travel_err16,
              "addr75 error = actual - target");
        check($signed(dut.curve_readouts[352+16 +: 16])<=16'sd12 &&
              $signed(dut.curve_readouts[352+16 +: 16])>=-16'sd12,
              "travel endpoint error within 12 codes (~4 deg)");

        delta=-16'sd3;
        force dut.travel_req_r=1'b1; force dut.travel_counts_r=-16'sd600;
        @(negedge clk); @(negedge clk);
        release dut.travel_req_r; release dut.travel_counts_r;
        ct;ct;
        check(dut.travel_mode && !dut.travel_done,"second travel re-arms");
        check(!dut.spin_parked,"second travel un-parks");
        check($signed(dut.travel_progress)<32'sd40,"progress restarts from zero");
        check(dut.travel_target===-32'sd600,"second target = -600 codes");
        i=0; while(!dut.travel_finish && i<2000) begin ct; i=i+1; end
        check(dut.travel_finish,"second travel finishes");
        delta=16'sd0;
        i=0; while(!(dut.spin_parked && dut.travel_done) && i<400) begin ct; i=i+1; end
        check(dut.travel_done && dut.travel_count===8'd2,"second completion counted");
        check(dut.u_position.origin===encoder,"second park rebases again");
        check($signed(dut.curve_readouts[352+16 +: 16])<=16'sd12 &&
              $signed(dut.curve_readouts[352+16 +: 16])>=-16'sd12,
              "second endpoint error within 12 codes");

        delta=16'sd3;
        force dut.travel_req_r=1'b1; force dut.travel_counts_r=16'sd900;
        @(negedge clk); @(negedge clk);
        release dut.travel_req_r; release dut.travel_counts_r;
        ct;ct;outers(2);
        delta=16'sd0;
        theta=16'd3095+16'd240;ct;ct;ct;outers(2);
        check(dut.spin_tilt_latch,"travel pulls the setpoint on rod tilt");
        check(dut.spin_target===32'sd0,"tilt: latched command is zero");
        check(dut.travel_aborted,"tilt: aborted flag set");
        check(!dut.travel_finish,"tilt is not the target being reached");
        theta=16'd3095;ct;ct;outers(2);
        check(!dut.spin_tilt_latch,"rod recovers: setpoint allowed again");
        delta=16'sd3;
        i=0; while(!dut.travel_finish && i<2000) begin ct; i=i+1; end
        check(dut.travel_finish,"travel resumes and reaches the target");
        delta=16'sd0;
        i=0; while(!(dut.spin_parked && dut.travel_done) && i<400) begin ct; i=i+1; end
        check(dut.travel_count===8'd3,"third completion counted");
        check(dut.u_position.origin===encoder,"third park rebases again");
        ct;ct;
        check(dut.curve_readouts[353]===1'b1,"addr75 aborted bit stays for this move");
        check($signed(dut.curve_readouts[352+16 +: 16])<=16'sd12 &&
              $signed(dut.curve_readouts[352+16 +: 16])>=-16'sd12,
              "tilt recovery still lands within 12 codes");

        release dut.u_swing.state;
        release dut.pos_q16;

        $display("ALL PASS spin_mode checks=%0d",checks);$finish;
    end
    initial begin #20000000;$fatal(1,"timeout");end
endmodule
