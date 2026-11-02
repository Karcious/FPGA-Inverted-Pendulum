`timescale 1ns / 1ps

`include "dbg_pkg.vh"

module pend_top #(
    parameter integer CLK_FREQ = 50_000_000,

    parameter integer BAUD     = 460800,

    parameter integer CTRL_HZ  = 1_000,
    parameter integer POS_INIT = 10_000,
    parameter integer DSH      = 10
)(

    input  wire        clk,
    input  wire        sw1_n,
    input  wire        sw2_n,
    input  wire        sw3_n,
    input  wire        sw4_n,
    input  wire        rst_n,

    input  wire        uart_rx_pin,
    output wire        uart_tx_pin,

    output wire        adc_clk,
    output wire        adc_oe_n,
    input  wire [9:0]  adc_data_in,
    input  wire        adc_otr,

    input  wire        enc1_a,
    input  wire        enc1_b,

    output wire        pwm,
    output wire        an1,
    output wire        an2,

    output wire        pwm_b,
    output wire        bn1,
    output wire        bn2,

    output wire [3:0]  led
);

    localparam [15:0] POS_INIT_W = POS_INIT;

    wire [52*32-1:0]  param_flat;

    wire [4*16-1:0]   probe_data;
    wire              cap_armed;
    wire              cap_done;

    localparam integer TICK_DIV = CLK_FREQ / CTRL_HZ;

    reg [31:0] tick_cnt;
    reg        ctrl_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt  <= 32'd0;
            ctrl_tick <= 1'b0;
        end else if (tick_cnt == TICK_DIV - 1) begin
            tick_cnt  <= 32'd0;
            ctrl_tick <= 1'b1;
        end else begin
            tick_cnt  <= tick_cnt + 1'b1;
            ctrl_tick <= 1'b0;
        end
    end

    wire [11:0] theta_raw;
    wire [11:0] theta_clean;
    wire        adc_valid;
    wire        otr_seen;

    adc_angle_filter u_angle_filter (
        .clk(clk), .rst_n(rst_n), .sample_valid(adc_valid),
        .sample(theta_raw), .filtered(theta_clean)
    );

    adc_capture u_adc (
        .clk        (clk),
        .rst_n      (rst_n),
        .adc_clk    (adc_clk),
        .adc_oe_n   (adc_oe_n),
        .adc_data_in(adc_data_in),
        .adc_otr    (adc_otr),
        .theta_raw  (theta_raw),
        .adc_valid  (adc_valid),
        .otr_seen   (otr_seen)
    );

    wire [31:0] trim_theta = param_flat[`P_TRIM_THETA*32 +: 32];

    reg [15:0] theta;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) theta<=0;
        else theta<={4'd0, theta_clean} + trim_theta[15:0];
    end

    wire signed [31:0] enc_pos;
    wire signed [15:0] enc_delta;
    wire               enc_dir;
    wire               enc_fault_tick;
    wire               enc_fault_seen;

    encoder_decode #(
        .CW       (32),
        .POS_INIT (POS_INIT)
    ) u_enc (
        .clk        (clk),
        .rst_n      (rst_n),
        .enc_a      (enc1_a),
        .enc_b      (enc1_b),
        .sample_en  (ctrl_tick),
        .pos        (enc_pos),
        .delta      (enc_delta),
        .dir        (enc_dir),
        .fault_tick (enc_fault_tick),
        .fault_seen (enc_fault_seen)
    );

    wire signed [15:0] pos_off;
    wire signed [31:0] position_error;
    wire position_fault;
    wire zero_long_press, zero_mark_pulse;
    wire mode_close;
    wire session;
    wire auto_balance;
    wire outer_run;
    wire [2:0] swing_state;
    wire signed [15:0] swing_drive;
    wire signed [31:0] position_target;
    wire signed [31:0] motion_target_ref;
    wire motion_traveling, motion_recovering;
    wire motion_key_arrival,curve_settled_entry,arrival_rebase;

    wire signed [17:0] motion_leg_pos;
    wire               motion_leg_active;
    wire profile_arrival;reg profile_done_previous;

    reg  spin_parked, spin_park_req;

    wire signed [15:0] motion_arrival_error;
    wire [7:0]         motion_arrival_count;

    wire travel_abandon_rebase;
    assign arrival_rebase=motion_key_arrival || curve_settled_entry || profile_arrival ||
                          spin_park_req || travel_abandon_rebase;
    wire motion_home_leg;

    wire position_origin_latched;
    reg signed [31:0] position_target_r;
    position_feedback #(.ORIGIN_INIT(POS_INIT), .LATCH_ON_ENTRY(1)) u_position (
        .clk(clk), .rst_n(rst_n), .active(outer_run), .session(session),
        .encoder(enc_pos), .target(position_target_r),
        .re_zero(zero_mark_pulse || arrival_rebase),
        .relative_pos(pos_off), .error_q16(position_error), .fault(position_fault),
        .origin_latched(position_origin_latched),

        .leg_active(motion_leg_active), .leg_pos(motion_leg_pos)
    );

    localparam [31:0] MODE_COAST = 32'd0;
    localparam [31:0] MODE_OPEN  = 32'd1;
    localparam [31:0] MODE_CLOSE = 32'd2;

    localparam [31:0] MODE_SPIN  = 32'd4;

    wire [31:0] p_openloop = param_flat[`P_OPENLOOP *32 +: 32];
    wire [31:0] p_dutytest = param_flat[`P_DUTY_TEST*32 +: 32];
    wire [31:0] motor_en   = param_flat[`P_MOTOR_EN  *32 +: 32];

    wire [31:0] filt_sel  = param_flat[`P_FILT_SEL*32 +: 32];

    reg mode_dec_open, mode_dec_close, mode_dec_auto, mode_dec_coast, mode_dec_spin;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_dec_open <=1'b0; mode_dec_close<=1'b0; mode_dec_auto<=1'b0;
            mode_dec_coast<=1'b0; mode_dec_spin <=1'b0;
        end else begin
            mode_dec_open <= (p_openloop == MODE_OPEN);
            mode_dec_close<= (p_openloop == MODE_CLOSE);

            mode_dec_auto <= (p_openloop == 32'd3) || (p_openloop == MODE_SPIN);
            mode_dec_coast<= (p_openloop == MODE_COAST);
            mode_dec_spin <= (p_openloop == MODE_SPIN);
        end
    end
    wire mode_open = mode_dec_open;

    wire spin_mode = mode_dec_spin;
    wire mode_auto = mode_dec_auto;

    reg mode_dec_close_d;
    reg [7:0] handoff_ms;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_dec_close_d <= 1'b0;
            handoff_ms       <= 8'd0;
        end else begin
            mode_dec_close_d <= mode_dec_close;
            if (mode_dec_close_d && !mode_dec_close && mode_dec_auto)
                handoff_ms <= 8'd200;
            else if (ctrl_tick && handoff_ms != 8'd0)
                handoff_ms <= handoff_ms - 1'b1;
        end
    end
    wire mode_handoff = (handoff_ms != 8'd0);

    wire motion_active = mode_auto && (swing_state == 3'd4);

    wire pos_rebase = zero_mark_pulse ||
                      (position_origin_latched && motion_active) ||
                      travel_abandon_rebase;
    assign mode_close = mode_dec_close || auto_balance;

    assign outer_run = mode_dec_close || motion_active;

    assign session = mode_dec_close || mode_auto;
    reg [31:0] adc_age;
    reg adc_seen;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin adc_age<=TICK_DIV; adc_seen<=0; end
        else if(adc_valid) begin adc_age<=0;adc_seen<=1;end
        else if(adc_age<TICK_DIV) adc_age<=adc_age+1'b1;
    end
    wire sensor_fresh = adc_seen && adc_age<TICK_DIV && !enc_fault_seen;
    wire sensor_ok = sensor_fresh && theta_raw>4 && theta_raw<4088;
    wire swing_entered,swing_confirmed,swing_gap_seen,swing_gap_active;
    wire signed [15:0] swing_pos;
    wire gains_valid = $signed(param_flat[`P_KP_BAL*32 +: 32])>0 &&
                       $signed(param_flat[`P_KD_BAL*32 +: 32])>=0 &&
                       $signed(param_flat[`P_KP_ARM*32 +: 32])<0 &&
                       $signed(param_flat[`P_KD_ARM*32 +: 32])<0 &&
                       motor_en==2;

    wire safe;
    wire [3:0] swing_fault;
    swing_once u_swing (
        .clk(clk),.rst_n(rst_n),.tick(ctrl_tick),.request(mode_auto),
        .balance_safe(safe || position_fault),.sensor_valid(sensor_ok),.gains_valid(gains_valid),
        .raw_angle(theta_raw),.sensor_fresh(sensor_fresh),
        .entered(swing_entered),.confirmed(swing_confirmed),
        .gap_seen(swing_gap_seen),.gap_active(swing_gap_active),.telemetry_pos(swing_pos),
        .theta(theta),.center(param_flat[`P_CENTER_ANGLE*32 +: 16]),
        .hang(param_flat[`P_HANG_ANGLE*32 +: 16]),.position(enc_pos),.delta(enc_delta),
        .pulse_duty(param_flat[`P_SWING_DUTY*32 +: 32]),
        .pulse_ms(param_flat[`P_SWING_MS*32 +: 32]),.timeout_ms(param_flat[`P_SWING_TIMEOUT*32 +: 32]),
        .travel_max(param_flat[`P_SWING_TRAVEL*32 +: 32]),
        .switch_deg(param_flat[`P_THETA_SWITCH*32 +: 32]),.switch_dps(param_flat[`P_OMEGA_SWITCH*32 +: 32]),
        .step_target(param_flat[29*32+:32]),.capture_ms(param_flat[30*32+:32]),
        .step_kp(param_flat[12*32+:32]),.step_kd(param_flat[14*32+:32]),

        .brake_gain(param_flat[`P_WAITUP_BRAKE*32+:32]),

        .settle_gain(param_flat[`P_SETTLE_DAMP*32+:32]),

        .mode_handoff(mode_handoff),
        .fault_code(swing_fault),
        .balance(auto_balance),.drive(swing_drive),.state(swing_state)
    );

    localparam integer INNER_DIV = 1;
    localparam integer OUTER_DIV = 25;

    reg [5:0] inner_cnt;
    reg [5:0] outer_cnt;
    reg       inner_tick;
    reg       outer_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inner_cnt  <= 6'd0;
            outer_cnt  <= 6'd0;
            inner_tick <= 1'b0;
            outer_tick <= 1'b0;
        end else begin
            inner_tick <= 1'b0;
            outer_tick <= 1'b0;
            if (ctrl_tick) begin
                if (inner_cnt == INNER_DIV - 1) begin
                    inner_cnt  <= 6'd0;
                    inner_tick <= 1'b1;
                    if (outer_cnt == OUTER_DIV - 1) begin
                        outer_cnt  <= 6'd0;
                        outer_tick <= 1'b1;
                    end else begin
                        outer_cnt <= outer_cnt + 1'b1;
                    end
                end else begin
                    inner_cnt <= inner_cnt + 1'b1;
                end
            end
        end
    end

    wire [31:0] p_center = param_flat[`P_CENTER_ANGLE*32 +: 32];
    wire [31:0] p_tgtpos = param_flat[`P_TARGET_POS  *32 +: 32];

    always @(posedge clk or negedge rst_n)
      if(!rst_n) position_target_r<=0;
      else if(arrival_rebase) position_target_r<=0;
      else position_target_r<=position_target;
    assign position_target = mode_dec_close ? $signed(p_tgtpos) :
                             motion_active ? motion_target_ref :
                                             32'sd0;

    wire signed [16:0] motion_angle_error_w =
        $signed({1'b0, theta}) - $signed({1'b0, p_center[15:0]});
    wire signed [15:0] motion_angle_error_next =
        (motion_angle_error_w >  17'sd32767) ?  16'sh7fff :
        (motion_angle_error_w < -17'sd32768) ?  16'sh8000 :
                                                motion_angle_error_w[15:0];

    reg signed [15:0] motion_angle_error;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) motion_angle_error<=0;
        else motion_angle_error<=motion_angle_error_next;
    end

    reg  signed [19:0] vel_acc, vel_sum;
    wire [15:0] spin_angle_abs = motion_angle_error[15]
                               ? (16'd0 - motion_angle_error[15:0])
                               :  motion_angle_error[15:0];

    wire quiet_blocked,quiet_wake,evade_request,evade_reverse;
    wire signed [31:0] evade_speed;
    wire [255:0] tof_readouts;wire[127:0] quiet_readouts;
    reg signed[16:0] quiet_angle_q;
    wire curve_owned,curve_speed_mode,curve_holding,curve_hold_entry;
    wire signed [31:0] curve_velocity,curve_hold_actual;
    wire curve_wr_en;wire [7:0] curve_wr_addr;wire [31:0] curve_wr_data;
    wire signed [31:0] vel_q16;
    wire [511:0] curve_readouts_raw;reg [895:0] curve_readouts;
    reg curve_start_ready,curve_pause,curve_stopped;
    reg [31:0] key_command_reply;

    reg  travel_mode, travel_finish, travel_aborted, travel_park_pulse;

    reg  travel_started;
    reg  travel_req_r, travel_done;
    reg  signed [15:0] travel_counts_r;
    reg  signed [31:0] travel_target, travel_progress;
    reg  [7:0] travel_count;
    wire signed [31:0] travel_abs = travel_target[31] ? -travel_target : travel_target;
    wire signed [31:0] travel_dir_prog =
        travel_target[31] ? -travel_progress : travel_progress;
    wire signed [31:0] travel_left = travel_abs - travel_dir_prog;

    wire signed [31:0] travel_err32 = travel_progress - travel_target;
    wire signed [15:0] travel_err16 =
        (travel_err32 >  32'sd32767) ? 16'sh7fff :
        (travel_err32 < -32'sd32768) ? 16'sh8000 : travel_err32[15:0];

    wire spin_active = !curve_owned && (spin_mode || (travel_mode && !travel_done))
                       && motion_active && !spin_parked;

    wire vel_owns_arm = spin_active || (travel_mode && !travel_done);
    always @(posedge clk or negedge rst_n)
      if(!rst_n) begin curve_readouts<=0;curve_start_ready<=0;curve_pause<=0;curve_stopped<=0;end
      else begin
        curve_readouts<={quiet_readouts,tof_readouts,curve_readouts_raw};
        curve_readouts[480 +: 32]<=key_command_reply;

        curve_readouts[352 +: 32]<= travel_mode
            ? {travel_err16, travel_count, 6'd0, travel_aborted,
               spin_parked || travel_done}
            : {motion_arrival_error, motion_arrival_count, 7'd0, spin_parked};
        curve_start_ready<=motion_active && !spin_mode && spin_angle_abs<=120 &&
            vel_sum>=-4 && vel_sum<=4;
        if(!curve_owned) curve_pause<=0;
        else if(spin_angle_abs>=240 || curve_hold_actual>4194304 || curve_hold_actual< -4194304) curve_pause<=1;
        else if(spin_angle_abs<=160 && curve_hold_actual<=2097152 && curve_hold_actual>=-2097152) curve_pause<=0;
        curve_stopped<=vel_sum>=-1 && vel_sum<=1 && spin_angle_abs<=60 &&
                       curve_hold_actual<=1048576 && curve_hold_actual>=-1048576;
      end
    spin_trajectory u_curve(.evade_request(evade_request),.evade_reverse(evade_reverse),.evade_speed(evade_speed),.clk(clk),.rst_n(rst_n),.tick(ctrl_tick),.outer_tick(outer_tick),
      .enable(motion_active && !safe && !position_fault),.start_ready(curve_start_ready),
      .pause_request(curve_pause),.stopped(curve_stopped),.encoder(enc_pos),
      .wr_en(curve_wr_en),.wr_addr(curve_wr_addr),.wr_data(curve_wr_data),
      .owned(curve_owned),.speed_mode(curve_speed_mode),.holding(curve_holding),
      .velocity_q16(curve_velocity),.hold_actual_q16(curve_hold_actual),
      .settled_entry(curve_settled_entry),.measured_velocity(vel_q16),.hold_entry(curve_hold_entry),.readouts(curve_readouts_raw));

    reg signed [31:0] p_move_slew_r, p_prof_slew_r, p_move_acc_r, p_move_dec_r;
    reg signed [31:0] p_spin_w_r, p_spin_kp_r, p_spin_ki_r, p_spin_acc_r;
    reg signed [31:0] p_home_ramp_r;

    reg signed [31:0] p_travel_follow_r;

    reg signed [31:0] p_arm_max_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_move_slew_r<=32'sd0; p_prof_slew_r<=32'sd0; p_move_acc_r<=32'sd0;
            p_move_dec_r <=32'sd0;
            p_spin_w_r<=32'sd0; p_spin_kp_r<=32'sd0; p_spin_ki_r<=32'sd0;
            p_spin_acc_r<=32'sd0; p_home_ramp_r<=32'sd0; p_arm_max_r<=32'sd0;
            p_travel_follow_r<=32'sd0;
        end else begin
            p_move_slew_r <= $signed(param_flat[`P_MOVE_SLEW*32 +: 32]);
            p_prof_slew_r <= $signed(param_flat[`P_PROF_SLEW*32 +: 32]);
            p_move_acc_r  <= $signed(param_flat[`P_MOVE_ACC *32 +: 32]);
            p_move_dec_r  <= $signed(param_flat[`P_MOVE_DEC *32 +: 32]);
            p_spin_w_r    <= $signed(param_flat[`P_SPIN_W   *32 +: 32]);
            p_spin_kp_r   <= $signed(param_flat[`P_SPIN_KP  *32 +: 32]);
            p_spin_ki_r   <= $signed(param_flat[`P_SPIN_KI  *32 +: 32]);
            p_spin_acc_r  <= $signed(param_flat[`P_SPIN_ACC *32 +: 32]);
            p_home_ramp_r <= $signed(param_flat[`P_HOME_RAMP*32 +: 32]);
            p_arm_max_r   <= $signed(param_flat[`P_ARM_MAX  *32 +: 32]);
            p_travel_follow_r <= $signed(param_flat[`P_TRAVEL_FOLLOW*32 +: 32]);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin vel_acc<=20'sd0; vel_sum<=20'sd0; end
        else begin
            if      (ctrl_tick)  vel_acc <= vel_acc + $signed(enc_delta);
            else if (outer_tick) vel_acc <= 20'sd0;
            if      (outer_tick) vel_sum <= vel_acc;
        end
    end

    localparam signed [31:0] SPIN_W_MAX = 32'sd64;
    wire signed [31:0] spin_cmd_next =
        (p_spin_w_r >  SPIN_W_MAX) ?  SPIN_W_MAX :
        (p_spin_w_r < -SPIN_W_MAX) ? -SPIN_W_MAX : p_spin_w_r;

    reg signed [31:0] spin_cmd;
    always @(posedge clk or negedge rst_n)
      if(!rst_n) spin_cmd<=0;
      else spin_cmd<=spin_cmd_next;

    reg spin_tilt_latch;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                     spin_tilt_latch<=1'b0;
        else if (!spin_active)          spin_tilt_latch<=1'b0;
        else if (outer_tick) begin
            if      (spin_angle_abs >= 16'd240) spin_tilt_latch<=1'b1;
            else if (spin_angle_abs <= 16'd160) spin_tilt_latch<=1'b0;
        end
    end

    wire signed [31:0] spin_cmd_abs = spin_cmd[31] ? -spin_cmd : spin_cmd;
    wire signed [31:0] spin_step   = (p_spin_acc_r > 0) ? p_spin_acc_r : 32'sd1;

    reg  signed [31:0] travel_v, travel_stop;

    wire signed [31:0] travel_margin = {spin_step[29:0],2'b00} +
                                       (spin_step<<<1);
    reg signed [31:0] spin_cmd_abs_r;
    reg signed [31:0] travel_margin_r;
    reg signed [31:0] t1_v_next;
    reg signed [31:0] t1_stop_base;
    reg signed [31:0] t1_v_dec;
    reg signed [31:0] t1_stop_dec;
    reg signed [31:0] t1_v, t1_left;
    reg signed [31:0] t2_stop_next;
    reg               t1_need_slow, t2_can_accel;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spin_cmd_abs_r<=32'sd0; travel_margin_r<=32'sd0;
            t1_v_next<=32'sd0; t1_stop_base<=32'sd0; t1_v_dec<=32'sd0;
            t1_stop_dec<=32'sd0; t1_v<=32'sd0; t1_left<=32'sd0;
            t2_stop_next<=32'sd0; t1_need_slow<=1'b0; t2_can_accel<=1'b0;
        end else begin
            spin_cmd_abs_r <= spin_cmd[31] ? -spin_cmd : spin_cmd;
            travel_margin_r<= travel_margin;

            t1_v_next    <= ((spin_cmd_abs_r - travel_v) < spin_step) ? spin_cmd_abs_r
                                                                      : (travel_v + spin_step);
            t1_stop_base <= travel_stop + travel_margin_r;
            t1_v_dec     <= travel_v - spin_step;
            t1_stop_dec  <= travel_stop - travel_v - travel_margin_r;
            t1_v         <= travel_v;
            t1_left      <= travel_left;

            t1_need_slow <= (travel_stop > travel_left) ||
                            (travel_v > spin_cmd_abs_r);

            t2_stop_next <= t1_stop_base + t1_v_next;
            t2_can_accel <= ((t1_v_next > t1_v) &&
                             ((t1_stop_base + t1_v_next) <= t1_left)) ||
                            (t1_v==32'sd0 && t1_v_next>32'sd0);
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin travel_v <= 32'sd0; travel_stop <= 32'sd0; end
        else if (!travel_mode || travel_finish || !motion_active ||
                 curve_owned || spin_mode) begin
            travel_v <= 32'sd0; travel_stop <= 32'sd0;
        end else if (outer_tick) begin
            if (t1_need_slow) begin

                if (travel_v > spin_step) begin
                    travel_stop <= t1_stop_dec;
                    travel_v    <= t1_v_dec;
                end else begin
                    travel_stop <= 32'sd0;
                    travel_v    <= 32'sd0;
                end
            end else if (t2_can_accel) begin
                travel_stop <= t2_stop_next;
                travel_v    <= t1_v_next;
            end
        end
    end

    wire signed [31:0] travel_cmd =
        (travel_finish || !travel_mode) ? 32'sd0 :
        (travel_target[31] ? -travel_v : travel_v);
    wire signed [31:0] spin_cmd_eff = travel_mode ? travel_cmd : spin_cmd;
    wire signed [31:0] spin_target = spin_tilt_latch ? 32'sd0 : spin_cmd_eff;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            travel_mode<=1'b0; travel_finish<=1'b0; travel_aborted<=1'b0;
            travel_done<=1'b0; travel_target<=32'sd0; travel_progress<=32'sd0;
            travel_count<=8'd0; travel_started<=1'b0;
        end else begin

            if (!motion_active || curve_owned || spin_mode ||
                motion_key_arrival || curve_settled_entry || profile_arrival) begin
                travel_mode<=1'b0; travel_finish<=1'b0; travel_aborted<=1'b0;
                travel_done<=1'b0; travel_progress<=32'sd0; travel_target<=32'sd0;
                travel_started<=1'b0;
                if (!motion_active) travel_count<=8'd0;

            end else if (travel_req_r) begin
                travel_mode<=1'b1; travel_finish<=1'b0; travel_aborted<=1'b0;
                travel_done<=1'b0; travel_progress<=32'sd0; travel_started<=1'b0;
                travel_target<={{16{travel_counts_r[15]}},travel_counts_r};
            end else if (travel_mode) begin

                if (ctrl_tick && !travel_done)
                    travel_progress<=travel_progress+$signed(enc_delta);
                if (outer_tick) travel_started<=1'b1;

                if (!travel_finish && (travel_left<=32'sd3 ||
                    (travel_started && travel_v==32'sd0 &&
                     travel_stop==32'sd0 && travel_left<=32'sd16))) travel_finish<=1'b1;
                if (spin_tilt_latch && !travel_finish) travel_aborted<=1'b1;
                if (travel_park_pulse) begin
                    travel_done<=1'b1; travel_count<=travel_count+1'b1;
                end
            end
        end
    end

    reg travel_abandoned, motion_active_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin travel_abandoned<=1'b0; motion_active_prev<=1'b0; end
        else begin
            motion_active_prev <= motion_active;
            if (travel_abandon_rebase) travel_abandoned <= 1'b0;
            else if (!motion_active && travel_mode && !travel_done)
                travel_abandoned <= 1'b1;
        end
    end
    assign travel_abandon_rebase =
        motion_active && !motion_active_prev && travel_abandoned;

    reg signed [31:0] spin_target_q;
    always @(posedge clk or negedge rst_n)
      if (!rst_n) spin_target_q <= 32'sd0;
      else        spin_target_q <= spin_target;
    reg signed [31:0] spin_ref;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            spin_ref <= 32'sd0;
        else if (!spin_active) spin_ref <= 32'sd0;
        else if (outer_tick) begin
            if (spin_ref < spin_target_q)
                spin_ref <= ((spin_target_q - spin_ref) <= spin_step)
                          ? spin_target_q : (spin_ref + spin_step);
            else if (spin_ref > spin_target_q)
                spin_ref <= ((spin_ref - spin_target_q) <= spin_step)
                          ? spin_target_q : (spin_ref - spin_step);
        end
    end

    localparam integer PARK_TICKS = 4;

    localparam integer PARK_VEL_MAX = 3;
    wire park_vel_ok = (vel_sum >= -PARK_VEL_MAX) && (vel_sum <= PARK_VEL_MAX);
    reg [2:0] spin_park_count;
    reg       spin_has_run, spin_park_settle;

    wire spin_stop_req = travel_mode ? travel_finish : (p_spin_w_r == 32'sd0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spin_parked<=1'b0; spin_has_run<=1'b0; spin_park_req<=1'b0;
            spin_park_count<=3'd0;
            spin_park_settle<=1'b0; travel_park_pulse<=1'b0;
        end else begin

            spin_park_req<=1'b0; travel_park_pulse<=1'b0;
            if ((!spin_mode && !travel_mode) || !motion_active || curve_owned) begin
                spin_parked<=1'b0; spin_has_run<=1'b0;
                spin_park_count<=3'd0; spin_park_settle<=1'b0;
            end else if (!spin_stop_req) begin
                spin_parked<=1'b0; spin_has_run<=1'b1;
                spin_park_count<=3'd0; spin_park_settle<=1'b0;
            end else if (spin_parked) begin

            end else if (spin_park_settle) begin

                spin_parked<=1'b1; spin_park_settle<=1'b0;

            end else if (outer_tick) begin

                if (spin_has_run && spin_ref==32'sd0 &&
                    (travel_mode ? (spin_angle_abs<=16'd240)
                                 : (spin_angle_abs<=16'd60)) &&
                    park_vel_ok) begin
                    if (spin_park_count>=PARK_TICKS-1) begin
                        spin_park_req<=1'b1; spin_park_settle<=1'b1; spin_park_count<=3'd0;

                        if (travel_mode) travel_park_pulse<=1'b1;
                    end else spin_park_count<=spin_park_count+1'b1;
                end else begin
                    spin_park_count<=3'd0;
                end
            end
        end
    end

    wire signed [19:0] vel_clamped = (vel_sum >  20'sd1024) ?  20'sd1024 :
                                     (vel_sum < -20'sd1024) ? -20'sd1024 : vel_sum;
    assign vel_q16      = $signed(vel_clamped) <<< 16;
    wire signed [31:0] spin_ref_q16 = spin_ref <<< 16;

    reg spin_was_active;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) spin_was_active<=1'b0;
        else        spin_was_active<=spin_active;
    end
    wire spin_entry = spin_active && !spin_was_active;

    wire zero_mark_allowed_next = mode_dec_coast ||
        (motion_active && sensor_ok && !safe && !position_fault &&
         $signed(motion_angle_error)>=-16'sd60 && $signed(motion_angle_error)<=16'sd60 &&
         $signed(enc_delta)>=-16'sd1 && $signed(enc_delta)<=16'sd1);
    reg zero_mark_allowed;
    always @(posedge clk or negedge rst_n)
      if(!rst_n) zero_mark_allowed<=0;
      else zero_mark_allowed<=zero_mark_allowed_next;

    wire remote_zero_wr = curve_wr_en && curve_wr_addr==8'd79 &&
                          curve_wr_data==32'd3 && zero_mark_allowed;
    reg remote_zero_req;
    always @(posedge clk or negedge rst_n)
      if(!rst_n) remote_zero_req<=1'b0;
      else       remote_zero_req<=remote_zero_wr;
    assign zero_mark_pulse = (zero_long_press && zero_mark_allowed) || remote_zero_req;
    reg zero_marked;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) zero_marked<=0;
        else if (zero_mark_pulse) zero_marked<=1;
    end

    wire signed [31:0] theta_q16  = $signed({16'd0, theta}) << 16;
    wire signed [31:0] pos_q16    = -position_error;
    wire signed [31:0] center_q16 = $signed({16'd0, p_center[15:0]}) << 16;

    wire spin_stopping = spin_active && spin_stop_req && (spin_ref == 32'sd0);
    wire spin_hold_req = spin_stopping && spin_has_run;
    reg  signed [31:0] spin_hold_q16;
    reg                spin_hold_valid, spin_hold_seen, spin_hold_entry;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spin_hold_q16<=32'sd0; spin_hold_valid<=1'b0;
            spin_hold_seen<=1'b0;  spin_hold_entry<=1'b0;
        end else begin
            if (!spin_hold_req) begin
                spin_hold_valid<=1'b0; spin_hold_seen<=1'b0;
            end else if (!spin_hold_valid) begin

                spin_hold_q16<=pos_q16;
                spin_hold_valid<=1'b1;
            end
            spin_hold_entry<=spin_hold_req && spin_hold_valid && !spin_hold_seen;
            if (spin_hold_req && spin_hold_valid) spin_hold_seen<=1'b1;
        end
    end
    wire spin_hold = spin_hold_req && spin_hold_valid;

    wire signed [31:0] outer_target = 32'sd0;

    reg mode_run;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) mode_run <= 1'b0;
        else        mode_run <= mode_close;
    end

    safety_guard u_safety (
        .clk    (clk),
        .rst_n  (rst_n),
        .en     (ctrl_tick),
        .run    (mode_run),
        .theta  (theta),
        .center (p_center[15:0]),
        .dpos   (enc_delta),
        .safe   (safe)
    );

    reg pid_clear;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pid_clear <= 1'b1;
        else        pid_clear <= !mode_close || safe || position_fault;
    end

    reg outer_clear;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) outer_clear <= 1'b1;
        else        outer_clear <= !outer_run || safe || position_fault;
    end

    reg [5:0] position_zero_hold;
    reg position_pid_clear;
    reg position_resume_pending;
    wire position_resume_clear = position_resume_pending && outer_tick && !position_pid_clear;
    reg position_pid_run;

    wire position_pid_rst_n = rst_n && position_pid_run;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) position_resume_pending<=1'b1;
        else if(position_pid_clear) position_resume_pending<=1'b1;
        else if(outer_tick) position_resume_pending<=1'b0;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            position_zero_hold<=0; position_pid_clear<=1; position_pid_run<=0;
        end
        else begin

            if (quiet_wake || zero_mark_pulse || arrival_rebase || spin_entry || curve_hold_entry || spin_hold_entry) begin
                position_zero_hold<=6'd32; position_pid_run<=0;
            end else if (outer_clear || quiet_blocked) begin

                position_zero_hold<=0; position_pid_run<=0;
            end else if (position_zero_hold!=0) begin
                position_zero_hold<=position_zero_hold-1'b1;
                if (position_zero_hold==1) position_pid_run<=1;
            end else begin
                position_pid_run<=1;
            end
            position_pid_clear<=outer_clear || quiet_blocked || quiet_wake || zero_mark_pulse || arrival_rebase || spin_entry || curve_hold_entry ||
                                spin_hold_entry || (position_zero_hold!=0);
        end
    end

    wire signed [31:0] ang_out;
    wire signed [31:0] outer_out;
    wire signed [31:0] kp_ang_eff, ki_ang_eff, kd_ang_eff;
    wire signed [31:0] kp_pos_eff, ki_pos_eff, kd_pos_eff;
    wire [9:0] kp_ang_scale, kd_ang_scale;
    wire signed [32:0] ec_ang_filtered;
    wire fuzzy_sample_done;

    wire [15:0] duty_max16 = param_flat[`P_DUTY_MAX*32 +: 16];
    wire [15:0] dm_safe    = (duty_max16 > 16'd8191) ? 16'd8191 : duty_max16;

    reg  signed [31:0] pid_out_max;
    reg  signed [31:0] pid_out_min;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pid_out_max <= 32'sd0;
            pid_out_min <= 32'sd0;
        end else begin
            pid_out_max <= $signed({16'd0, dm_safe}) << 16;
            pid_out_min <= -($signed({16'd0, dm_safe}) << 16);
        end
    end

    wire [15:0] arm_max16 = (duty_max16 > 16'd750) ? 16'd750 : duty_max16;

    wire [15:0] arm_cap16 = (p_arm_max_r > 0) ? p_arm_max_r[15:0] : 16'd0;
    wire [15:0] arm_max16_eff =
        (motion_home_leg && arm_cap16 != 16'd0 && arm_cap16 < arm_max16)
        ? arm_cap16 : arm_max16;

    reg signed [31:0] arm_out_max;
    reg signed [31:0] arm_out_min;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arm_out_max <= 32'sd0;
            arm_out_min <= 32'sd0;
        end else begin
            arm_out_max <=  $signed({16'd0, arm_max16_eff}) << 16;
            arm_out_min <= -($signed({16'd0, arm_max16_eff}) << 16);
        end
    end

    fuzzy_pid_q16 #(
        .E_STEP_SHIFT  (2),
        .EC_STEP_SHIFT (0)
    ) u_pid_ang (
        .fuzzy_enable(param_flat[`P_FILT_SEL*32+8]),
        .clk     (clk),
        .rst_n   (rst_n),
        .en      (inner_tick),
        .clear   (pid_clear),
        .target  (center_q16),
        .actual  (theta_q16),
        .d_input (32'sd0),
        .d_clear (1'b0),
        .kp      (param_flat[`P_KP_BAL*32 +: 32]),
        .ki      (32'sd0),
        .kd      (param_flat[`P_KD_BAL*32 +: 32]),
        .out_max (pid_out_max),
        .out_min (pid_out_min),

        .en_dlpf   (filt_sel[0]),
        .en_dspan2 (filt_sel[1]),

        .en_dspan4 (filt_sel[5]),
        .lpf_shift (2'd0),
        .out     (ang_out),
        .kp_eff  (kp_ang_eff),
        .ki_eff  (ki_ang_eff),
        .kd_eff  (kd_ang_eff),
        .kp_scale_dbg(kp_ang_scale),
        .kd_scale_dbg(kd_ang_scale),
        .ec_filtered_dbg(ec_ang_filtered),
        .sample_done(fuzzy_sample_done)
    );

    localparam [15:0] SCHED_MAX_MS = 16'd5000;

    wire [15:0] pa_raw = param_flat[`P_PHASE_A_MS*32 +: 16];
    wire [15:0] pb_raw = param_flat[`P_PHASE_B_MS*32 +: 16];
    wire [15:0] phase_a_ms = (pa_raw > SCHED_MAX_MS) ? SCHED_MAX_MS : pa_raw;
    wire [15:0] phase_b_ms = (pb_raw > SCHED_MAX_MS) ? SCHED_MAX_MS : pb_raw;

    reg [15:0] phase_ms;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) phase_ms <= 16'd0;

        else if (outer_clear) phase_ms <= 16'd0;
        else if (ctrl_tick && phase_ms != 16'hFFFF) phase_ms <= phase_ms + 1'b1;
    end

    wire sched_on   = (phase_a_ms != 16'd0);
    wire in_phase_a = sched_on && (phase_ms < phase_a_ms);
    wire in_phase_b = sched_on && (phase_ms >= phase_a_ms) &&
                      (phase_ms < (phase_a_ms + phase_b_ms));

    reg phase_a_q, phase_b_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin phase_a_q <= 1'b0; phase_b_q <= 1'b0; end
        else        begin phase_a_q <= in_phase_a; phase_b_q <= in_phase_b; end
    end

    wire [31:0] kp_arm_sel = phase_a_q ? param_flat[`P_GAIN_A_KP*32 +: 32] :
                             phase_b_q ? param_flat[`P_GAIN_B_KP*32 +: 32] :
                                         param_flat[`P_KP_ARM  *32 +: 32];
    wire [31:0] kd_arm_sel = phase_a_q ? param_flat[`P_GAIN_A_KD*32 +: 32] :
                             phase_b_q ? param_flat[`P_GAIN_B_KD*32 +: 32] :
                                         param_flat[`P_KD_ARM  *32 +: 32];

    wire signed [32:0] kp_arm_twice = $signed({kp_arm_sel[31],kp_arm_sel}) <<< 1;
    wire signed [31:0] kp_arm_fast =
        kp_arm_twice>33'sd2147483647 ? 32'sh7fffffff :
        kp_arm_twice< -33'sd2147483648 ? 32'sh80000000 : kp_arm_twice[31:0];
    reg signed [31:0] kp_arm_control, ki_arm_control;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin kp_arm_control<=0; ki_arm_control<=0; end
        else begin
            kp_arm_control <= (motion_traveling && !motion_recovering) ?
                               kp_arm_fast : $signed(kp_arm_sel);
            ki_arm_control <= motion_traveling ? 32'sd0 :
                               $signed(param_flat[`P_KI_ARM*32 +: 32]);
        end
    end

    wire signed [31:0] outer_target_sel = spin_hold ? 32'sd0 :
                                          spin_active ? spin_ref_q16 : outer_target;
    wire signed [31:0] outer_actual_sel = spin_hold ? spin_hold_q16 :
                                          spin_active ? vel_q16 :
                                          curve_owned ? curve_hold_actual : pos_q16;

    localparam integer SPIN_STOP_BOOST = 2;
    wire signed [32:0] kp_spin_boost_w = $signed({p_spin_kp_r[31], p_spin_kp_r})
                                       * SPIN_STOP_BOOST;
    wire signed [31:0] spin_kp_stop =
        (kp_spin_boost_w >  33'sd2147483647) ?  32'sh7fffffff :
        (kp_spin_boost_w < -33'sd2147483648) ?  32'sh80000000 :
                                                kp_spin_boost_w[31:0];

    wire signed [31:0] outer_kp_sel     = spin_hold   ? kp_arm_control
                                                      : spin_active ? (spin_stopping ? spin_kp_stop : p_spin_kp_r)
                                                      : kp_arm_control;
    wire signed [31:0] outer_ki_sel     = spin_hold   ? ki_arm_control
                                                      : spin_active ? p_spin_ki_r  : ki_arm_control;
    wire signed [31:0] outer_kd_sel     = spin_hold   ? kd_arm_sel
                                                      : spin_active ? 32'sd0       : kd_arm_sel;

    wire signed [31:0] encoder_d_q16 = {enc_pos[15:0],16'd0};

    wire outer_d_measured = motion_active && (!spin_active || spin_hold) && !curve_owned;

    reg outer_d_was_measured;

    always @(posedge clk or negedge rst_n)
        if(!rst_n) outer_d_was_measured<=1'b0;
        else if(outer_tick && !position_pid_clear) outer_d_was_measured<=outer_d_measured;
    wire outer_d_clear = outer_d_measured != outer_d_was_measured;
    wire signed [31:0] outer_d_input =
        outer_d_measured ? -encoder_d_q16 :
                                                        outer_target_sel-outer_actual_sel;

    wire outer_fuzzy_sel = param_flat[`P_FILT_SEL*32+8] && !spin_active;

    fuzzy_pid_q16 #(
        .E_STEP_SHIFT  (4),
        .EC_STEP_SHIFT (1),

        .TUNE_ENABLE   (0),
        .SEPARATE_D    (1)
    ) u_pid_pos (
        .fuzzy_enable(outer_fuzzy_sel),
        .clk     (clk),
        .rst_n   (position_pid_rst_n),
        .en      (outer_tick && !position_pid_clear),
        .clear   (position_pid_clear || position_resume_clear),

        .target  (outer_target_sel),
        .actual  (outer_actual_sel),
        .d_input (outer_d_input),
        .d_clear (outer_d_clear),

        .kp      (outer_kp_sel),
        .ki      (outer_ki_sel),

        .kd      (outer_kd_sel),
        .out_max (arm_out_max),
        .out_min (arm_out_min),

        .en_dlpf   (filt_sel[2]),
        .en_dspan2 (filt_sel[3]),

        .en_dspan4 (filt_sel[4]),
        .lpf_shift (filt_sel[7:6]),
        .out     (outer_out),
        .kp_eff  (kp_pos_eff),
        .ki_eff  (ki_pos_eff),
        .kd_eff  (kd_pos_eff)
    );

    reg signed [31:0] outer_src;
    always @(posedge clk or negedge rst_n)
      if(!rst_n) outer_src<=0;
      else outer_src<=motion_recovering ? ($signed(outer_out) >>> 2) : outer_out;
    wire signed [31:0] ang_duty32 = (ang_out - outer_src) >>> 16;
    wire signed [15:0] pid_duty   = (ang_duty32 >  32'sd32767) ?  16'sd32767 :
                                    (ang_duty32 < -32'sd32768) ? -16'sd32768 :
                                                                  ang_duty32[15:0];

    wire signed [15:0] duty_cmd = (safe || position_fault || quiet_blocked) ? 16'sd0 :
                                  mode_open  ? p_dutytest[15:0] :
                                  mode_close ? pid_duty        :
                                  mode_auto  ? swing_drive     : 16'sd0;

    reg signed [15:0] duty_cmd_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) duty_cmd_q <= 16'sd0;
        else        duty_cmd_q <= duty_cmd;
    end

    wire signed [15:0] duty_a = motor_en[0] ? duty_cmd_q : 16'sd0;
    wire signed [15:0] duty_b = motor_en[1] ? duty_cmd_q : 16'sd0;

    wire signed [15:0] duty_applied;
    wire               blanking;
    wire [15:0] friction_comp;
    wire [3:0] friction_shift;
    wire friction_moving;
    friction_state u_friction (
        .clk(clk), .rst_n(rst_n), .tick(ctrl_tick),
        .active(mode_close && !safe && !position_fault),
        .enable(param_flat[`P_FRICTION_MODE*32 +: 32] == 32'd1),
        .delta(enc_delta), .static_comp(param_flat[`P_DUTY_MIN*32 +: 16]),
        .moving_comp(param_flat[`P_MOVING_COMP*32 +: 16]),

        .shape_sel(param_flat[`P_COMP_SHIFT*32 +: 4]),
        .compensation(friction_comp), .comp_shift(friction_shift), .moving(friction_moving)
    );

    pwm_gen #(
        .CLK_FREQ (CLK_FREQ),
        .PWM_HZ   (20_000),
        .DSH      (DSH), .COMP_SHAPING(1)
    ) u_pwm_a (
        .clk          (clk),
        .rst_n        (rst_n),
        .duty_in      (duty_a),
        .duty_max     (param_flat[`P_DUTY_MAX*32 +: 16]),
        .duty_min     (friction_comp), .comp_shift(friction_shift),
        .pwm          (pwm),
        .an1          (an1),
        .an2          (an2),
        .duty_applied (),
        .blanking     ()
    );

    pwm_gen #(
        .CLK_FREQ (CLK_FREQ),
        .PWM_HZ   (20_000),
        .DSH      (DSH), .COMP_SHAPING(1)
    ) u_pwm_b (
        .clk          (clk),
        .rst_n        (rst_n),
        .duty_in      (duty_b),
        .duty_max     (param_flat[`P_DUTY_MAX*32 +: 16]),
        .duty_min     (friction_comp), .comp_shift(friction_shift),
        .pwm          (pwm_b),
        .an1          (bn1),
        .an2          (bn2),
        .duty_applied (duty_applied),
        .blanking     (blanking)
    );

    wire [15:0] ch_theta = theta;
    wire [15:0] ch_delta = enc_delta;
    wire [15:0] ch_pos   = pos_off;
    wire [15:0] ch_duty  = duty_applied;

    reg [63:0] probe_q;
    reg probe_tick;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin probe_q<=0;probe_tick<=0;end
        else begin
            probe_q<={ch_duty,ch_pos,ch_delta,ch_theta};
            probe_tick<=ctrl_tick;
        end
    end
    assign probe_data = probe_q;

    wire signed [31:0] ec_ang_dbg32 =
        (ec_ang_filtered > 33'sd2147483647) ? 32'sh7fffffff :
        (ec_ang_filtered < -33'sd2147483648) ? 32'sh80000000 : ec_ang_filtered[31:0];
    wire [31:0] fuzzy_flags = {11'd0,filt_sel[8],kd_ang_scale,kp_ang_scale};
    reg [127:0] fuzzy_debug_data;
    reg fuzzy_sample_pending;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin fuzzy_sample_pending<=0; fuzzy_debug_data<=0; end
        else begin
            fuzzy_sample_pending<=fuzzy_sample_done;
            if(fuzzy_sample_pending)
                fuzzy_debug_data<={fuzzy_flags,ec_ang_dbg32,kd_ang_eff,kp_ang_eff};
        end
    end

    wire button_press;
    wire key_neg_press, physical_key_neg;
    reg remote_key_neg, remote_key_pos;
    assign key_neg_press = physical_key_neg || remote_key_neg;
    wire key_zero_press;
    wire key_pos_press, physical_key_pos;
    assign key_pos_press = physical_key_pos || remote_key_pos;
    wire signed [31:0] motion_target_cmd;
    wire motion_ready;
    wire motion_moving;
    wire motion_paused;
    wire motion_reached;
    wire motion_homing, motion_home_fault;
    wire profile_start_a, profile_start_b;
    wire profile_command_valid, profile_hold, profile_busy, profile_paused;
    wire profile_done, profile_fault, profile_id, motion_at_target;
    wire [2:0] profile_segment;
    wire signed [31:0] profile_target, profile_slew;

    wire signed [15:0] cmd4_counts = curve_wr_data[31:16];

    wire signed [15:0] cmd5_counts = curve_wr_data[31:16];
    reg remote_move_req_r;
    reg signed [15:0] remote_move_counts_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_command_reply<=0; remote_key_neg<=0; remote_key_pos<=0;
            remote_move_req_r<=1'b0; remote_move_counts_r<=16'sd0;
            travel_req_r<=1'b0; travel_counts_r<=16'sd0;
        end else begin
            remote_key_neg<=0; remote_key_pos<=0; remote_move_req_r<=1'b0;

            if (travel_req_r && (travel_mode || !motion_active || curve_owned ||
                                 spin_mode || motion_key_arrival ||
                                 curve_settled_entry || profile_arrival))
                travel_req_r<=1'b0;
            if (curve_wr_en && curve_wr_addr==8'd79) begin
                key_command_reply[31:8]<=key_command_reply[31:8]+1'b1;
                key_command_reply[7:0]<={5'b0,curve_wr_data[2:0]};
                if (curve_wr_data[15:0]==16'd4) begin

                    remote_move_req_r<=1'b0;
                    if (curve_owned)
                        key_command_reply[6:4]<=3'd2;
                    else if (!motion_active || spin_mode || safe || position_fault ||
                             motion_home_fault || (travel_mode && !travel_done) ||
                             motion_leg_active)
                        key_command_reply[6:4]<=3'd1;
                    else if (!motion_ready)
                        key_command_reply[6:4]<=3'd4;

                    else if (cmd4_counts==16'sd0 || cmd4_counts>16'sd8191 || cmd4_counts< -16'sd8191)
                        key_command_reply[6:4]<=3'd3;
                    else begin
                        key_command_reply[7]<=1'b1;

                        remote_move_req_r<=1'b1;
                        remote_move_counts_r<=cmd4_counts;
                    end
                end else if (curve_wr_data[15:0]==16'd5) begin

                    if (curve_owned)
                        key_command_reply[6:4]<=3'd2;
                    else if (!motion_active || spin_mode || safe || position_fault ||
                             motion_home_fault || (travel_mode && !travel_done) ||
                             motion_leg_active)
                        key_command_reply[6:4]<=3'd1;
                    else if (!motion_ready)
                        key_command_reply[6:4]<=3'd4;
                    else if (cmd5_counts==16'sd0)
                        key_command_reply[6:4]<=3'd3;
                    else if (spin_cmd==32'sd0)
                        key_command_reply[6:4]<=3'd5;
                    else begin
                        key_command_reply[7]<=1'b1;
                        travel_req_r<=1'b1;
                        travel_counts_r<=cmd5_counts;
                    end
                end else if (curve_wr_data==3) begin

                    key_command_reply[7]<=remote_zero_wr;
                    key_command_reply[6:4]<=remote_zero_wr ? 3'd0 : 3'd5;
                end else if (curve_wr_data!=1 && curve_wr_data!=2)
                    key_command_reply[6:4]<=3'd3;
                else if (curve_owned)
                    key_command_reply[6:4]<=3'd2;
                else if (!motion_active || spin_mode || safe || position_fault ||
                         motion_home_fault || (travel_mode && !travel_done) ||
                         motion_leg_active)
                    key_command_reply[6:4]<=3'd1;
                else if (!motion_ready)
                    key_command_reply[6:4]<=3'd4;
                else begin
                    key_command_reply[7]<=1'b1;
                    remote_key_neg<=(curve_wr_data==1);
                    remote_key_pos<=(curve_wr_data==2);
                end
            end
        end
    end

    start_button u_start_button(.clk(clk), .rst_n(rst_n), .tick(ctrl_tick),
        .sw1_n(sw1_n), .press(button_press), .long_press());
    start_button #(.LONG_PRESS_MS(2000)) u_sw2_button(
        .clk(clk), .rst_n(rst_n), .tick(ctrl_tick),
        .sw1_n(sw2_n), .press(physical_key_neg), .long_press(profile_start_a));
    start_button #(.LONG_PRESS_MS(2000)) u_sw3_button(
        .clk(clk), .rst_n(rst_n), .tick(ctrl_tick),
        .sw1_n(sw3_n), .press(key_zero_press), .long_press(zero_long_press));
    start_button #(.LONG_PRESS_MS(2000)) u_sw4_button(
        .clk(clk), .rst_n(rst_n), .tick(ctrl_tick),
        .sw1_n(sw4_n), .press(physical_key_pos), .long_press(profile_start_b));

    motion_profile_sequencer u_profile (
        .clk(clk), .rst_n(rst_n), .tick(ctrl_tick),
        .enable(!curve_owned && motion_ready && motion_active && !safe && !position_fault && !motion_home_fault),
        .cancel(curve_owned || key_zero_press || key_neg_press || key_pos_press),
        .re_zero(zero_mark_pulse),
        .start_a(profile_start_a), .start_b(profile_start_b),
        .reached(motion_reached), .at_target(motion_at_target),
        .command_valid(profile_command_valid), .command_target(profile_target),
        .command_slew(profile_slew), .hold_motion(profile_hold),
        .busy(profile_busy), .paused(profile_paused), .done(profile_done),
        .fault(profile_fault), .profile_id(profile_id), .segment(profile_segment)
    );

    key_motion_target u_key_motion (
        .clk         (clk),
        .rst_n       (rst_n),
        .tick        (ctrl_tick),

        .active      (motion_active && !curve_owned && !vel_owns_arm),
        .balance_confirmed(motion_active && !curve_owned && !vel_owns_arm),

        .re_zero     (pos_rebase || profile_arrival || curve_settled_entry || spin_park_req),
        .fault       (safe || position_fault),
        .key_neg     (key_neg_press),
        .key_zero    (key_zero_press),
        .key_pos     (key_pos_press),

        .remote_move_req   (remote_move_req_r),
        .remote_move_counts(remote_move_counts_r),
        .profile_request(profile_command_valid), .profile_target(profile_target),
        .profile_slew(profile_slew), .hold_motion(profile_hold),
        .angle_error (motion_angle_error),
        .actual_pos  (pos_off),

        .actual_leg  (motion_leg_pos),
        .leg_active  (motion_leg_active),
        .dpos        (enc_delta),

        .slew_sel      (p_move_slew_r),
        .prof_slew_sel (p_prof_slew_r),
        .accel_sel     (p_move_acc_r),

        .decel_sel     (p_move_dec_r),

        .home_slew_sel (p_home_ramp_r),

        .travel_follow_sel (p_travel_follow_r),
        .target_ref  (motion_target_ref),
        .target_cmd  (motion_target_cmd),
        .ready       (motion_ready),
        .moving      (motion_moving),
        .paused      (motion_paused),
        .reached     (motion_reached),
        .homing      (motion_homing),
        .home_fault  (motion_home_fault),
        .at_target   (motion_at_target),
        .traveling   (motion_traveling),
        .recovering  (motion_recovering),
        .home_leg    (motion_home_leg),.key_arrival(motion_key_arrival),

        .arrival_error(motion_arrival_error),
        .arrival_count(motion_arrival_count)
    );

    always @(posedge clk or negedge rst_n)
      if(!rst_n) profile_done_previous<=0;
      else profile_done_previous<=profile_done;
    assign profile_arrival=profile_done && !profile_done_previous && !curve_owned && motion_active;

    reg [31:0] control_status_q;
    always @(posedge clk or negedge rst_n)
      if(!rst_n) control_status_q<=0;
      else control_status_q<={profile_segment,profile_id,profile_fault,profile_done,profile_busy,
                         motion_home_fault,motion_homing,zero_marked,
                         motion_ready,motion_moving,(motion_paused || profile_paused),motion_reached,
                         swing_gap_active,swing_gap_seen,swing_confirmed,swing_entered,swing_fault,otr_seen,enc_fault_seen,sensor_ok,blanking,
                         friction_moving,position_fault,safe,swing_state};

    always @(posedge clk or negedge rst_n)
      if(!rst_n)quiet_angle_q<=0;else quiet_angle_q<=$signed(theta_q16-center_q16) >>> 16;
    quiet_guard u_quiet(.clk(clk),.rst_n(rst_n),.tick(ctrl_tick),
      .eligible(mode_auto && !spin_mode && !travel_mode && motion_active && motion_ready && motion_at_target && !motion_moving && !motion_homing && !curve_owned && !profile_busy && !safe && !position_fault && !motion_home_fault && sensor_ok),
      .angle(quiet_angle_q),.delta(enc_delta),.wr_en(curve_wr_en),.wr_addr(curve_wr_addr),.wr_data(curve_wr_data),.blocked(quiet_blocked),.wake(quiet_wake),.readouts(quiet_readouts));
    tof_receiver u_tof(.clk(clk),.rst_n(rst_n),.tick(ctrl_tick),
      .permit(mode_auto && motion_active && !spin_mode && !travel_mode && !safe && !position_fault),
      .evade_request(evade_request),.reverse(evade_reverse),.speed(evade_speed),
      .wr_en(curve_wr_en),.wr_addr(curve_wr_addr),.wr_data(curve_wr_data),.readouts(tof_readouts));

    dbg_top #(
        .BUTTON_ENABLE(1),

        .PW(8),

        .REV      (8'h19),

        .N_PARAM  (52),

        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_dbg (
        .button_press(button_press),
        .trajectory_readouts(curve_readouts),.trajectory_wr_en(curve_wr_en),
        .trajectory_wr_addr(curve_wr_addr),.trajectory_wr_data(curve_wr_data),
        .control_status(control_status_q),
        .clk          (clk),
        .rst_n        (rst_n),
        .uart_rx_pin  (uart_rx_pin),
        .uart_tx_pin  (uart_tx_pin),
        .param_flat   (param_flat),
        .sample_en    (probe_tick),
        .probe_data   (probe_data),
        .fuzzy_debug_data(fuzzy_debug_data),
        .cap_armed    (cap_armed),
        .cap_done     (cap_done)
    );

    reg [24:0] hb_cnt;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) hb_cnt <= 25'd0;
        else        hb_cnt <= hb_cnt + 1'b1;

    reg        rx_sync0, rx_sync1;
    reg [15:0] rx_edges;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync0 <= 1'b1;
            rx_sync1 <= 1'b1;
            rx_edges <= 16'd0;
        end else begin
            rx_sync0 <= uart_rx_pin;
            rx_sync1 <= rx_sync0;
            if (rx_sync1 && !rx_sync0)
                rx_edges <= rx_edges + 1'b1;
        end
    end

    assign led[0] = hb_cnt[24];
    assign led[1] = enc_fault_seen;

    assign led[2] = cap_done;
    assign led[3] = (rx_edges != 0);

endmodule
