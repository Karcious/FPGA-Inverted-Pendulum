`timescale 1ns/1ps

module furuta_plant #(

    parameter integer N_SUB = 4
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               load,
    input  wire signed [31:0] init_deg_milli,
    input  wire signed [31:0] init_thd_milli,
    input  wire signed [31:0] init_phd_milli,
    input  wire               step,
    input  wire signed [15:0] duty_in,
    input  wire               noise_en,
    output reg  [11:0]        theta_raw,
    output reg  signed [31:0] enc_pos,
    output reg  signed [15:0] enc_delta
);
    localparam real PI = 3.141592653589793;

    localparam real CENTER       = 3095.0;
    localparam real SPAN_180     = 2078.0;
    localparam real CODE_PER_RAD = SPAN_180/PI;
    localparam real ENC_CPR      = 1040.0;
    localparam real RAD_PER_CNT  = 2.0*PI/ENC_CPR;

    localparam real R_ARM = 0.156;
    localparam real L_C   = 0.070;
    localparam real M_ROD = 0.050;
    localparam real L_EFF = 0.106;
    localparam real K2    = L_EFF*L_C;

    localparam real TAU_STALL = 0.647;
    localparam real OMEGA_NL  = 61.0;
    localparam real TAU_C_ARM = 0.030;
    localparam real TAU_S_ARM = 0.075;
    localparam real J_ARM     = 2.9e-3;
    localparam real B_THETA   = 0.3;
    localparam real TAU_C_ROD = 0.0;
    localparam real B_EMF     = TAU_STALL/OMEGA_NL;
    localparam real GRAV      = 9.81;
    localparam real DUTY_FULL = 1024.0;
    localparam real OMEGA_EPS = 0.02;
    localparam real ADC_NOISE_CODES = 0.59;
    localparam real DT_MS = 1.0e-3;

    real th, thd, ph, phd;
    real du_now;
    real k1[0:3], k2[0:3], k3[0:3], k4[0:3];
    integer seed;
    integer i, j, tc, np, dd;

    function real th_soft;
        input real x;
        real e;
        begin
            if      (x >  20.0) th_soft =  1.0;
            else if (x < -20.0) th_soft = -1.0;
            else begin
                e = $exp(2.0*x);
                th_soft = (e - 1.0)/(e + 1.0);
            end
        end
    endfunction

    function real soft_coulomb;
        input real tcx, vel;
        begin
            soft_coulomb = tcx * th_soft(vel/0.5);
        end
    endfunction

    function integer rnd;
        input real x;
        begin
            rnd = (x >= 0.0) ? $rtoi(x + 0.5) : -$rtoi(-x + 0.5);
        end
    endfunction

    task derivs;
        input  real ti, tdi, pii, pdi, du;
        output real dth, dthd, dph, dphd;
        real s, c, A11, b1, tau_drive, tau_arm, tau_cpl, A22, a12, det, stk;
        begin
            s = $sin(ti);  c = $cos(ti);

            A11 = M_ROD*K2;
            b1  = M_ROD*GRAV*L_C*s
                + M_ROD*K2*s*c*pdi*pdi
                - A11*B_THETA*tdi
                - soft_coulomb(TAU_C_ROD, tdi);

            tau_drive = TAU_STALL*du/DUTY_FULL;
            tau_arm   = tau_drive - B_EMF*pdi - soft_coulomb(TAU_C_ARM, pdi);

            tau_cpl   = - 2.0*M_ROD*K2*s*c*tdi*pdi
                        - M_ROD*R_ARM*L_C*s*tdi*tdi;

            A22 = J_ARM + M_ROD*R_ARM*R_ARM + M_ROD*K2*s*s;
            a12 = -M_ROD*R_ARM*L_C*c;

            dth = tdi;  dph = pdi;

            stk = tau_drive + tau_cpl;
            if ((pdi < OMEGA_EPS) && (pdi > -OMEGA_EPS) &&
                (stk <= TAU_S_ARM) && (stk >= -TAU_S_ARM)) begin
                dthd = b1/A11;  dphd = 0.0;
            end else begin
                det = A11*A22 - a12*a12;
                if ((det < 1.0e-18) && (det > -1.0e-18)) det = 1.0e-18;
                dthd = (b1*A22 - a12*(tau_arm + tau_cpl))/det;
                dphd = (A11*(tau_arm + tau_cpl) - b1*a12)/det;
            end
        end
    endtask

    task rk4;
        input real dt;
        begin
            derivs(th, thd, ph, phd, du_now, k1[0],k1[1],k1[2],k1[3]);
            derivs(th + dt/2.0*k1[0], thd + dt/2.0*k1[1],
                   ph + dt/2.0*k1[2], phd + dt/2.0*k1[3], du_now,
                   k2[0],k2[1],k2[2],k2[3]);
            derivs(th + dt/2.0*k2[0], thd + dt/2.0*k2[1],
                   ph + dt/2.0*k2[2], phd + dt/2.0*k2[3], du_now,
                   k3[0],k3[1],k3[2],k3[3]);
            derivs(th + dt*k3[0], thd + dt*k3[1],
                   ph + dt*k3[2], phd + dt*k3[3], du_now,
                   k4[0],k4[1],k4[2],k4[3]);
            th  = th  + dt/6.0*(k1[0] + 2.0*k2[0] + 2.0*k3[0] + k4[0]);
            thd = thd + dt/6.0*(k1[1] + 2.0*k2[1] + 2.0*k3[1] + k4[1]);
            ph  = ph  + dt/6.0*(k1[2] + 2.0*k2[2] + 2.0*k3[2] + k4[2]);
            phd = phd + dt/6.0*(k1[3] + 2.0*k2[3] + 2.0*k3[3] + k4[3]);
        end
    endtask

    task sense;
        real cf, a;
        begin
            cf = CENTER + th*CODE_PER_RAD;
            if (noise_en) begin

                a = 0.0;
                for (j = 0; j < 12; j = j + 1)
                    a = a + ($random(seed) / 2147483648.0);
                cf = cf + ADC_NOISE_CODES*(a - 6.0);
            end
            tc = rnd(cf);
            if (tc < 135)  tc = 135;
            if (tc > 4095) tc = 4095;
            theta_raw <= tc[11:0] - 12'd135;
            np = rnd(-(ph/RAD_PER_CNT));
            dd = np - enc_pos;
            if (dd >  32767) dd =  32767;
            if (dd < -32768) dd = -32768;
            enc_pos   <= np;
            enc_delta <= dd[15:0];
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            th = 0.0; thd = 0.0; ph = 0.0; phd = 0.0;
            seed = 32'h1234_5678;
            theta_raw <= 12'd0; enc_pos <= 32'sd0; enc_delta <= 16'sd0;
        end else if (load) begin
            th  = init_deg_milli * PI / 180000.0;
            thd = init_thd_milli / 1000.0;
            ph  = 0.0;
            phd = init_phd_milli / 1000.0;
            enc_pos   <= 32'sd0;
            enc_delta <= 16'sd0;
            tc = rnd(CENTER + th*CODE_PER_RAD);
            if (tc < 135)  tc = 135;
            if (tc > 4095) tc = 4095;
            theta_raw <= tc[11:0] - 12'd135;
        end else if (step) begin
            du_now = duty_in;
            for (i = 0; i < N_SUB; i = i + 1)
                rk4(DT_MS / N_SUB);
            sense;
        end
    end
endmodule

module tb_spin_plant;
    localparam real PI = 3.141592653589793;
    localparam integer CLKS_PER_MS = 256;

    reg clk = 0;
    always #10 clk = ~clk;

    reg rst_n = 0, tick = 0, pstep = 0, load = 0;
    reg [52*32-1:0] params = 0;

    reg signed [31:0] init_deg_milli = 2000;
    reg signed [31:0] init_phd_milli = 0;
    reg signed [31:0] init_thd_milli = 0;
    reg               noise_en = 1'b1;

    wire signed [15:0] duty_applied = dut.duty_applied;
    wire [11:0]        pl_theta_raw;
    wire signed [31:0] pl_enc_pos;
    wire signed [15:0] pl_enc_delta;

    wire pwm,an1,an2,pwm_b,bn1,bn2,adc_clk,adc_oe,tx;
    wire [3:0] led;

    pend_top dut(.sw1_n(1'b1),.sw2_n(1'b1),.sw3_n(1'b1),.sw4_n(1'b1),
        .clk(clk),.rst_n(rst_n),.uart_rx_pin(1'b1),.uart_tx_pin(tx),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe),.adc_data_in(10'd740),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.pwm(pwm),.an1(an1),.an2(an2),
        .pwm_b(pwm_b),.bn1(bn1),.bn2(bn2),.led(led));

    furuta_plant plant(.clk(clk),.rst_n(rst_n),.load(load),
        .init_deg_milli(init_deg_milli),.init_thd_milli(init_thd_milli),
        .init_phd_milli(init_phd_milli),
        .step(pstep),.duty_in(duty_applied),.noise_en(noise_en),
        .theta_raw(pl_theta_raw),.enc_pos(pl_enc_pos),.enc_delta(pl_enc_delta));

    integer arg_ms, arg_mode, arg_w, arg_kp, arg_ki, arg_ramp, arg_degm;
    integer arg_noise, arg_ol, arg_jump, arg_dbg, arg_fall, arg_dummy;

    integer arg_pause;

    integer arg_travel, arg_travel_ms, tr_pending;

    integer arg_parkfree;

    integer arg_poshold;
    reg signed [63:0] ph_ref_acc, ph_act_acc;
    wire signed [63:0] ph_err_now    = ph_ref_acc - ph_act_acc;
    wire signed [63:0] ph_hold64     = ph_act_acc - ph_ref_acc;
    wire signed [31:0] ph_hold_actual = ph_hold64[31:0];
    wire signed [31:0] ph_err32      = ph_err_now[31:0];
    integer m, n_stat;
    integer park_seen = 0, safe_seen = 0, latch_seen = 0, st_prev = -1;
    integer fin_seen = 0, done_seen = 0;
    real    rod_max, rod_sq, sat_cnt, safe_cnt, danger_cnt;
    real    dg, adg, cf_cum, ph_prev, fdeg;
    integer fi, hit[0:5];

    task step_ms;
        begin
            @(negedge clk); tick = 1;
            @(negedge clk); tick = 0;
            repeat (CLKS_PER_MS - 4) @(negedge clk);
            pstep = 1; @(negedge clk); pstep = 0;
            @(negedge clk);
        end
    endtask

    initial begin

        params[ 0*32+:32]=32'h0002_000A;
        params[ 1*32+:32]=32'd1000;
        params[ 2*32+:32]=32'd20000;
        params[ 3*32+:32]=32'h0003_8000;
        params[ 4*32+:32]=32'h0064_0000;
        params[ 5*32+:32]=32'hFFFE_8000;
        params[ 6*32+:32]=32'hFFFF_F852;
        params[ 7*32+:32]=32'd30;
        params[ 8*32+:32]=32'd600;
        params[ 9*32+:32]=32'd15;
        params[10*32+:32]=32'd130;
        params[11*32+:32]=32'd1024;
        params[12*32+:32]=32'h0004_0000;
        params[13*32+:32]=32'd0;
        params[14*32+:32]=32'h0001_999a;
        params[15*32+:32]=32'd135;
        params[16*32+:32]=32'd0;

        params[18*32+:32]=32'd3095;
        params[19*32+:32]=32'd2;
        params[20*32+:32]=32'hFFE0_0000;
        params[23*32+:32]=32'd50;
        params[24*32+:32]=32'hFFFF_FDA8;
        params[25*32+:32]=32'd600;
        params[26*32+:32]=32'd1500;
        params[27*32+:32]=32'd520;
        params[28*32+:32]=32'd1017;
        params[29*32+:32]=32'd340;
        params[30*32+:32]=32'd55;
        params[32*32+:32]=32'd0;
        params[33*32+:32]=32'hFFFE_8000;
        params[34*32+:32]=32'hFFE0_0000;
        params[35*32+:32]=32'hFFFE_8000;
        params[36*32+:32]=32'hFFE0_0000;
        params[37*32+:32]=32'd0;
        params[38*32+:32]=32'd0;
        params[39*32+:32]=32'h0002_0000;
        params[40*32+:32]=32'h0000_2000;
        params[41*32+:32]=32'd3787;
        params[42*32+:32]=32'd0;
        params[43*32+:32]=32'd0;
        params[44*32+:32]=32'hFFF8_0000;
        params[45*32+:32]=32'd0;
        params[46*32+:32]=32'd4;

        arg_ms = 2000; arg_mode = 3; arg_w = 0; arg_kp = -8; arg_ki = 0;
        arg_ramp = 4;  arg_degm = 2000; arg_noise = 1; arg_ol = 0; arg_jump = 1;
        arg_dbg = 0; arg_fall = 0; arg_pause = 0;
        arg_travel = 0; arg_travel_ms = 1500; tr_pending = 0;
        arg_parkfree = 0; arg_poshold = 0;
        arg_dummy = $value$plusargs("poshold=%d", arg_poshold);
        arg_dummy = $value$plusargs("park_free=%d", arg_parkfree);
        arg_dummy = $value$plusargs("travel=%d", arg_travel);
        arg_dummy = $value$plusargs("travel_ms=%d", arg_travel_ms);
        arg_dummy = $value$plusargs("pause_ms=%d", arg_pause);
        arg_dummy = $value$plusargs("ms=%d",    arg_ms);
        arg_dummy = $value$plusargs("mode=%d",  arg_mode);
        arg_dummy = $value$plusargs("w=%d",     arg_w);
        arg_dummy = $value$plusargs("kp=%d",    arg_kp);
        arg_dummy = $value$plusargs("ki=%d",    arg_ki);
        arg_dummy = $value$plusargs("ramp=%d",  arg_ramp);
        arg_dummy = $value$plusargs("degm=%d",  arg_degm);
        arg_dummy = $value$plusargs("noise=%d", arg_noise);
        arg_dummy = $value$plusargs("ol=%d",    arg_ol);
        arg_dummy = $value$plusargs("jump=%d",  arg_jump);
        arg_dummy = $value$plusargs("dbg=%d",   arg_dbg);
        arg_dummy = $value$plusargs("fall=%d",  arg_fall);
        init_deg_milli = arg_degm;
        noise_en       = (arg_noise != 0);
        params[16*32+:32] = arg_mode;
        params[43*32+:32] = arg_w;
        params[44*32+:32] = arg_kp <<< 16;

        params[45*32+:32] = (arg_ki * 65536) / 1000;
        params[46*32+:32] = arg_ramp;

        force dut.param_flat = params;
        force dut.ctrl_tick  = tick;
        force dut.adc_valid  = 1'b1;
        force dut.enc_fault_seen = 1'b0;

        force dut.u_pwm_b.blank = 1'b0;

        force dut.theta_raw = pl_theta_raw;
        force dut.enc_pos   = pl_enc_pos;
        force dut.enc_delta = pl_enc_delta;
        if (arg_parkfree) force dut.park_vel_ok = 1'b1;

        ph_ref_acc = 0; ph_act_acc = 0;
        if (arg_poshold) begin
            force dut.outer_target_sel  = 32'sd0;
            force dut.outer_actual_sel  = ph_hold_actual;
            force dut.outer_kp_sel      = dut.kp_arm_control;
            force dut.outer_ki_sel      = dut.ki_arm_control;
            force dut.outer_kd_sel      = dut.kd_arm_sel;
            force dut.outer_d_measured  = 1'b0;
        end

        #1;
        repeat (4) @(negedge clk);
        rst_n = 1;
        @(negedge clk);
        load = 1; @(negedge clk); load = 0;
        repeat (6) @(negedge clk);
        if (arg_jump) force dut.u_swing.state = 3'd4;
        repeat (2) @(negedge clk);

        rod_max = 0.0; rod_sq = 0.0; sat_cnt = 0.0; safe_cnt = 0.0; danger_cnt = 0.0;
        ph_prev = plant.ph; cf_cum = 0.0; n_stat = 0;

        if (arg_ol != 0 || arg_fall) begin

            force plant.duty_in = arg_ol[15:0];
            noise_en = 1'b0;
            for (fi = 0; fi < 6; fi = fi + 1) hit[fi] = 0;
            for (m = 0; m < arg_ms; m = m + 1) begin
                @(negedge clk);
                pstep = 1; @(negedge clk); pstep = 0;
                @(negedge clk);
                if (arg_fall) begin
                    fdeg = plant.th*180.0/PI;
                    if (fdeg < 0.0) fdeg = -fdeg;
                    for (fi = 0; fi < 6; fi = fi + 1)
                        if (!hit[fi] && fdeg >= (fi+1)*30.0) begin
                            hit[fi] = 1;
                            $display("FALL deg=%0d ms=%0d", (fi+1)*30, m);
                        end
                end
            end
            if (!arg_fall)
            $display("OPENLOOP duty=%0d ms=%0d omega=%f rpm=%f turns=%f",
                     arg_ol, arg_ms, plant.phd, plant.phd*60.0/(2.0*PI),
                     plant.ph/(2.0*PI));
        end else begin

            for (m = 0; m < arg_ms; m = m + 1) begin

                if (arg_pause > 0 && m == arg_pause) begin
                    params[43*32+:32] = 32'sd0;
                    $display("PAUSE m=%0d mem43=0 arm_rpm=%f tilt=%0d",
                             m, plant.phd*60.0/(2.0*PI), $signed(dut.spin_angle_abs));
                end
                if (arg_travel != 0 && m == arg_travel_ms) begin
                    force dut.travel_counts_r = arg_travel[15:0];
                    force dut.travel_req_r    = 1'b1;
                    tr_pending = 2;
                    $display("TRAVEL_REQ m=%0d counts=%0d arm_rpm=%f",
                             m, arg_travel, plant.phd*60.0/(2.0*PI));
                end
                if (tr_pending > 0) begin
                    tr_pending = tr_pending - 1;
                    if (tr_pending == 0) begin
                        release dut.travel_req_r;
                        release dut.travel_counts_r;
                    end
                end

                if (arg_poshold) begin
                    ph_ref_acc = ph_ref_acc + $signed(dut.spin_ref) * 64'sd2621;
                end
                step_ms();
                if (arg_poshold) begin
                    ph_act_acc = ph_act_acc + ($signed(pl_enc_delta) <<< 16);
                end

                if (dut.spin_parked && !park_seen) begin
                    park_seen = 1;
                    $display("PARK m=%0d arm_rpm=%f tilt=%0d", m,
                             plant.phd*60.0/(2.0*PI), $signed(dut.spin_angle_abs));
                end
                if (dut.swing_state != st_prev) begin
                    $display("STATE m=%0d %0d->%0d arm_rpm=%f tilt=%0d safe=%b",
                             m, st_prev, dut.swing_state,
                             plant.phd*60.0/(2.0*PI), $signed(dut.spin_angle_abs), dut.safe);
                    st_prev = dut.swing_state;
                end
                if (dut.safe && !safe_seen) begin
                    safe_seen = 1;
                    $display("SAFE m=%0d arm_rpm=%f tilt=%0d", m,
                             plant.phd*60.0/(2.0*PI), $signed(dut.spin_angle_abs));
                end
                if (dut.travel_finish && !fin_seen) begin
                    fin_seen = 1;
                    $display("TRAVEL_FINISH m=%0d prog=%0d left=%0d arm_rpm=%f",
                             m, $signed(dut.travel_progress), $signed(dut.travel_left),
                             plant.phd*60.0/(2.0*PI));
                end
                if (dut.travel_done && !done_seen) begin
                    done_seen = 1;
                    $display("TRAVEL_DONE m=%0d prog=%0d arm_rpm=%f",
                             m, $signed(dut.travel_progress), plant.phd*60.0/(2.0*PI));
                end
                if (dut.spin_tilt_latch && !latch_seen) begin
                    latch_seen = 1;
                    $display("TILT_LATCH m=%0d arm_rpm=%f tilt=%0d", m,
                             plant.phd*60.0/(2.0*PI), $signed(dut.spin_angle_abs));
                end
                if (arg_dbg && (m % arg_dbg == 0 || m < 5))
                    $display("T%0d th=%f thd=%f | duty=%0d dc=%0d dq=%0d ang=%0d out=%0d | safe=%b posf=%b ab=%b mc=%b pc=%b st=%0d sOk=%b gOk=%b",
                             m, plant.th*180.0/PI, plant.thd,
                             $signed(duty_applied), $signed(dut.duty_cmd), $signed(dut.duty_cmd_q),
                             $signed(dut.ang_out), $signed(dut.outer_out),
                             dut.safe, dut.position_fault, dut.auto_balance, dut.mode_close,
                             dut.pid_clear, dut.swing_state, dut.sensor_ok, dut.gains_valid);
                    $display("   V%0d vel=%0d ref=%0d latch=%b ang_abs=%0d phd=%f",
                             m, $signed(dut.vel_sum), $signed(dut.spin_ref),
                             dut.spin_tilt_latch, $signed(dut.spin_angle_abs), plant.phd);
                    $display("   V%0d vel=%0d ref=%0d latch=%b ang_abs=%0d rot=%f",
                             m, $signed(dut.vel_sum), $signed(dut.spin_ref),
                             dut.spin_tilt_latch, $signed(dut.spin_angle_abs), plant.ph);
                    $display("   R%0d left=%0d prog=%0d tv=%0d fin=%b done=%b tm=%b act=%b park=%b abt=%b", m,
                             $signed(dut.travel_left), $signed(dut.travel_progress),
                             $signed(dut.travel_v), dut.travel_finish, dut.travel_done,
                             dut.travel_mode, dut.spin_active, dut.spin_parked,
                             dut.travel_aborted);

                if (m*4 >= arg_ms) begin
                    dg  = plant.th*180.0/PI;
                    adg = (dg >= 0.0) ? dg : -dg;
                    if (adg > rod_max) rod_max = adg;
                    rod_sq = rod_sq + adg*adg;
                    if (duty_applied > 1000 || duty_applied < -1000)
                        sat_cnt = sat_cnt + 1.0;
                    if (dut.safe) safe_cnt = safe_cnt + 1.0;
                    if (adg > 90.0) danger_cnt = danger_cnt + 1.0;
                    n_stat = n_stat + 1;
                    cf_cum = cf_cum + (plant.ph - ph_prev);
                    ph_prev = plant.ph;
                end
            end
            rod_sq = rod_sq / n_stat;
            if (arg_poshold)
                $display("POSHOLD ms=%0d ref_codes=%0d act_codes=%0d lag_codes=%f ref_rpm=%f",
                         arg_ms, $signed(ph_ref_acc[47:16]), $signed(ph_act_acc[47:16]),
                         $itor(ph_err32)/65536.0,
                         $itor($signed(dut.spin_ref))*40.0*60.0/1040.0);
            $display("RESULT mode=%0d w=%0d kp=%0d ki=%0d ramp=%0d ms=%0d degm=%0d noise=%0d rod_max=%f rod_rms=%f arm_rpm=%f turns=%f sat_pct=%f safe_pct=%f danger_pct=%f final_deg=%f",
                     arg_mode, arg_w, arg_kp, arg_ki, arg_ramp, arg_ms, arg_degm, arg_noise,
                     rod_max, $sqrt(rod_sq), plant.phd*60.0/(2.0*PI), cf_cum/(2.0*PI),
                     100.0*sat_cnt/n_stat, 100.0*safe_cnt/n_stat, 100.0*danger_cnt/n_stat,
                     plant.th*180.0/PI);
        end
        $display("ALL PASS spin_plant ms=%0d mode=%0d", arg_ms, arg_mode);
        $finish;
    end

    initial begin
        #1500000000;
        $display("TIMEOUT spin_plant");
        $fatal(1, "spin_plant timeout");
    end
endmodule
