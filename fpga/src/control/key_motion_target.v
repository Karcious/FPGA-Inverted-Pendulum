`timescale 1ns/1ps

module key_motion_target #(
    parameter signed [31:0] STEP_Q16       = 32'sd17039360,
    parameter signed [31:0] LIMIT_Q16      = 32'sd34078720,
    parameter signed [31:0] SLEW_Q16       = 32'sh00002000,
    parameter signed [31:0] HOME_SLEW_Q16  = 32'sh00000800,
    parameter integer ANGLE_GOOD_MAX       = 60,
    parameter integer TRAVEL_ANGLE_MAX     = 240,
    parameter integer TRAVEL_RESUME_MAX    = 160,
    parameter integer PREDICT_STOP_MAX     = 320,
    parameter integer PREDICT_RESUME_MAX   = 240,
    parameter integer SETTLE_ERROR_MAX     = 6,
    parameter integer READY_TICKS          = 300,
    parameter integer PAUSE_ERROR_MAX      = 16,
    parameter integer REACHED_ERROR_MAX    = 2,
    parameter integer DPOS_GOOD_MAX        = 1,
    parameter integer REACHED_TICKS        = 200,

    parameter integer KEY_SETTLE_TICKS     = 40,
    parameter integer HOME_TIMEOUT_TICKS   = 30000,
    parameter signed [31:0] MAX_SLEW_Q16   = 32'sd3787,

    parameter integer QUALIFY_MS_FAST      = 40,
    parameter integer QUALIFY_ERROR_FAST   = 32,

    parameter integer HOME_FOLLOW_MAX     = 64,

    parameter integer TRAVEL_FOLLOW_MAX   = 96

)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    tick,
    input  wire                    active,
    input  wire                    balance_confirmed,
    input  wire                    re_zero,
    input  wire                    fault,
    input  wire                    key_neg,
    input  wire                    key_zero,
    input  wire                    key_pos,

    input  wire                    remote_move_req,
    input  wire signed [15:0]      remote_move_counts,
    input  wire                    profile_request,
    input  wire signed [31:0]       profile_target,
    input  wire signed [31:0]       profile_slew,
    input  wire                    hold_motion,
    input  wire signed [15:0]      angle_error,
    input  wire signed [15:0]      actual_pos,

    input  wire signed [17:0]      actual_leg,
    input  wire signed [15:0]      dpos,

    input  wire signed [31:0]      slew_sel,
    input  wire signed [31:0]      prof_slew_sel,
    input  wire signed [31:0]      accel_sel,

    input  wire signed [31:0]      decel_sel,

    input  wire signed [31:0]      home_slew_sel,

    input  wire signed [31:0]      travel_follow_sel,
    output reg  signed [31:0]      target_ref,
    output reg  signed [31:0]      target_cmd,
    output reg                     ready,
    output wire                    moving,
    output wire                    paused,
    output reg                     reached,
    output wire                    homing,
    output reg                     home_fault,
    output wire                    at_target,
    output wire                    traveling,
    output wire                    recovering,

    output reg                     home_leg,

    output reg                     key_arrival,

    output reg  signed [15:0]      arrival_error,
    output reg  [7:0]              arrival_count,

    output wire                    leg_active
);

    wire home_ramp_en = (home_slew_sel > 0);

    wire [15:0] angle_bits = angle_error;
    wire [15:0] dpos_bits  = dpos;
    wire [15:0] angle_abs  = angle_bits[15] ? (16'd0 - angle_bits) : angle_bits;
    wire [15:0] dpos_abs   = dpos_bits[15]  ? (16'd0 - dpos_bits)  : dpos_bits;
    wire angle_good = (angle_abs <= ANGLE_GOOD_MAX);
    wire dpos_good  = (dpos_abs  <= DPOS_GOOD_MAX);

    wire signed [15:0] ref_integer = target_ref[31:16];

    reg  leg_long;

    reg travel_phase, recovery_hold;

    reg key_leg;

    wire leg_active_w = travel_phase && key_leg && leg_long;
    assign leg_active = leg_active_w;

    wire signed [17:0] actual_now = leg_active_w ? actual_leg
                                                 : {{2{actual_pos[15]}}, actual_pos};
    wire signed [18:0] follow_delta =
        {{3{ref_integer[15]}}, ref_integer} - {{1{actual_now[17]}}, actual_now};
    wire [18:0] follow_abs = follow_delta[18]
                           ? (19'd0 - follow_delta) : follow_delta;
    wire position_reached = (follow_abs <= REACHED_ERROR_MAX);

    wire [16:0] qualify_tol  = home_ramp_en ? QUALIFY_ERROR_FAST[16:0]
                                            : REACHED_ERROR_MAX[16:0];
    wire        qualify_pos  = (follow_abs <= qualify_tol);
    wire [15:0] qualify_need = home_ramp_en ? QUALIFY_MS_FAST[15:0]
                                            : READY_TICKS[15:0];

    wire        travel_follow_unknown = (^travel_follow_sel) === 1'bx;
    wire        travel_follow_ok      = !travel_follow_unknown &&
                                        (travel_follow_sel > 32'sd0) &&
                                        (travel_follow_sel <= 32'sd4096);
    wire [16:0] travel_follow_lim     = travel_follow_ok
                                      ? travel_follow_sel[16:0]
                                      : TRAVEL_FOLLOW_MAX[16:0];
    wire [16:0] follow_lim = (travel_phase && key_leg) ? travel_follow_lim
                           : home_leg                  ? HOME_FOLLOW_MAX[16:0]
                                                       : PAUSE_ERROR_MAX[16:0];
    wire follow_good = (follow_abs <= follow_lim);
    reg was_active, qualified;
    reg signed [15:0] previous_angle;
    reg angle_history_valid;
    wire signed [16:0] angle_delta =
        {angle_error[15],angle_error} - {previous_angle[15],previous_angle};
    wire [16:0] rate_abs = angle_delta[16] ? (17'd0-angle_delta) : angle_delta;
    wire outward = angle_history_valid &&
                   ((angle_error>0 && angle_delta>0) ||
                    (angle_error<0 && angle_delta<0));

    wire [21:0] predicted_angle = {6'd0,angle_abs} +
                                 (outward ? {1'b0,rate_abs,4'd0} : 22'd0);
    wire travel_unsafe = (angle_abs>=TRAVEL_ANGLE_MAX) ||
                         (predicted_angle>=PREDICT_STOP_MAX);
    wire recovery_good = (angle_abs<=TRAVEL_RESUME_MAX) &&
                          (predicted_angle<=PREDICT_RESUME_MAX);

    assign traveling = active && balance_confirmed && !fault && qualified &&
                        ready && !home_fault && travel_phase;
    assign recovering = traveling && (recovery_hold || travel_unsafe);
    wire advance_blocked = (traveling ? recovering : !angle_good) || !follow_good;

    wire actual_far = (actual_pos >  PAUSE_ERROR_MAX) ||
                      (actual_pos < -PAUSE_ERROR_MAX);

    wire signed [32:0] step_ext      = {STEP_Q16[31],  STEP_Q16};
    wire signed [32:0] neg_step_ext  = -step_ext;
    wire signed [32:0] limit_ext     = {LIMIT_Q16[31], LIMIT_Q16};
    wire signed [32:0] neg_limit_ext = -limit_ext;

    wire signed [31:0] pos_command =
        (step_ext > limit_ext)     ? LIMIT_Q16 :
        (step_ext < neg_limit_ext) ? -LIMIT_Q16 : step_ext[31:0];
    wire signed [31:0] neg_command =
        (neg_step_ext > limit_ext)     ? LIMIT_Q16 :
        (neg_step_ext < neg_limit_ext) ? -LIMIT_Q16 : neg_step_ext[31:0];

    wire key_any = key_zero || key_neg || key_pos;

    wire signed [31:0] key_command =
        (key_zero || (key_neg && key_pos)) ? 32'sd0 :
        key_neg ? neg_command : pos_command;

    localparam integer PENDING_MS = 2000;
    reg [1:0]  pending_key;
    reg [15:0] pending_age;
    wire pending_valid = (pending_key != 2'd0);
    wire signed [31:0] pending_command =
        (pending_key == 2'd3) ? 32'sd0 :
        (pending_key == 2'd1) ? neg_command : pos_command;

    wire signed [31:0] eff_command = key_any ? key_command : pending_command;

    wire eff_any = key_any || (pending_valid && !remote_move_req);
    wire pending_taken = !key_arrival && !re_zero && ready && balance_confirmed && !home_fault && eff_any;

    wire remote_taken  = !key_arrival && !re_zero && ready && balance_confirmed && !home_fault &&
                         remote_move_req && !eff_any && !leg_active_w;

    localparam signed [15:0] LEG_MIN_COUNTS = 16'sd520;
    wire remote_long = (remote_move_counts >  LEG_MIN_COUNTS) ||
                       (remote_move_counts < -LEG_MIN_COUNTS);

    wire signed [31:0] remote_command = {remote_move_counts, 16'd0};

    wire signed [32:0] ref_ext  = {target_ref[31], target_ref};
    wire signed [32:0] cmd_ext  = {target_cmd[31], target_cmd};
    reg signed [31:0] current_slew;
    wire signed [32:0] profile_ext = {profile_target[31],profile_target};
    wire signed [31:0] bounded_profile_target =
        profile_ext>limit_ext ? LIMIT_Q16 :
        profile_ext<neg_limit_ext ? -LIMIT_Q16 : profile_target;

    wire signed [31:0] slew_eff      = (slew_sel      > 0) ? slew_sel      : SLEW_Q16;

    wire signed [31:0] home_slew_eff = home_ramp_en ? home_slew_sel : slew_eff;
    wire signed [31:0] prof_slew_eff = (prof_slew_sel > 0) ? prof_slew_sel : MAX_SLEW_Q16;
    wire signed [31:0] accel_eff     = (accel_sel     > 0) ? accel_sel     : 32'sd0;

    wire signed [31:0] decel_eff     = (decel_sel     > 0) ? decel_sel     : accel_eff;

    wire               ramp_en       = ready && ((accel_eff != 0) || (decel_sel > 0));
    wire signed [31:0] bounded_profile_slew =
        profile_slew<1 ? 32'sd1 :
        profile_slew>prof_slew_eff ? prof_slew_eff : profile_slew;
    wire signed [32:0] distance_up   = cmd_ext - ref_ext;
    wire signed [32:0] distance_down = ref_ext - cmd_ext;

    localparam integer BRAKE_SHIFT = 8;
    wire signed [32:0] remain_abs = (distance_up > 0) ? distance_up : distance_down;

    wire signed [35:0] brake_zone_old = (decel_sel > 0)
        ? {current_slew[31], current_slew, 9'b0}
        : {current_slew[31], current_slew, {BRAKE_SHIFT{1'b0}}};

    reg braking_r;
    reg signed [31:0] key_tail_slew_r;
    wire signed [31:0] crawl_slew = (current_slew < HOME_SLEW_Q16) ? current_slew : HOME_SLEW_Q16;

    wire signed [32:0] tail_speed_wide = remain_abs >>> 6;
    wire signed [31:0] tail_floor = (crawl_slew<32'sd64) ? crawl_slew : 32'sd64;
    wire signed [31:0] key_tail_next =
        tail_speed_wide>$signed({crawl_slew[31],crawl_slew}) ? crawl_slew :
        tail_speed_wide<$signed({tail_floor[31],tail_floor}) ? tail_floor : tail_speed_wide[31:0];

    reg  signed [31:0] stop_v;
    reg  signed [39:0] stop_sum;
    reg        [17:0] stop_iter;
    reg                stop_done;
    reg  signed [31:0] stop_slew_q, stop_decel_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stop_v<=32'sd0; stop_sum<=40'sd0; stop_iter<=18'd0;
            stop_done<=1'b0; stop_slew_q<=32'sd0; stop_decel_q<=32'sd0;
        end else if (current_slew!=stop_slew_q || decel_eff!=stop_decel_q) begin
            stop_slew_q <= current_slew;
            stop_decel_q<= decel_eff;
            stop_v      <= (current_slew<=32'sd0)     ? 32'sd0 :
                           (current_slew>32'sh100000) ? 32'sh100000 : current_slew;
            stop_sum    <= 40'sd0;
            stop_iter   <= 18'd0;
            stop_done   <= 1'b0;
        end else if (!stop_done) begin
            if (stop_v>crawl_slew && decel_eff>32'sd0 && stop_iter<18'd65535) begin
                stop_sum  <= stop_sum + {{8{stop_v[31]}}, stop_v};
                stop_v    <= stop_v - decel_eff;
                stop_iter <= stop_iter + 18'd1;
            end else stop_done <= 1'b1;
        end
    end

    wire signed [39:0] taper_dist = {{2{crawl_slew[31]}}, crawl_slew, 6'b0};
    wire signed [39:0] brake_exact = stop_sum + taper_dist
                                   + {{6{current_slew[31]}}, current_slew, 2'b0};
    wire               braking_exact = ({{7{remain_abs[32]}}, remain_abs} <= brake_exact);
    wire               use_exact_brake = travel_phase && key_leg && stop_done;
    wire               braking_now = use_exact_brake ? braking_exact
                                                     : (remain_abs <= brake_zone_old);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin braking_r<=1'b0;key_tail_slew_r<=HOME_SLEW_Q16;end
        else begin
            braking_r <= (current_slew > 0) && braking_now;
            key_tail_slew_r <= key_tail_next;
        end
    end
    wire signed [31:0] slew_goal_next = (ramp_en && braking_r) ?
                                    ((travel_phase && key_leg) ? key_tail_slew_r : crawl_slew)
                                                          : current_slew;

    reg signed [31:0] slew_goal;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) slew_goal<=HOME_SLEW_Q16;
        else slew_goal<=slew_goal_next;

    reg  signed [31:0] slew_applied;

    wire signed [31:0] slew_used = (!ramp_en || slew_applied>current_slew) ? current_slew : slew_applied;
    wire signed [32:0] slew_ext  = {slew_used[31], slew_used};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                           slew_applied <= HOME_SLEW_Q16;

        else if (!ramp_en)                    slew_applied <= HOME_SLEW_Q16;
        else if (re_zero || !active || fault) slew_applied <= HOME_SLEW_Q16;
        else if (!was_active)                 slew_applied <= HOME_SLEW_Q16;

        else if ((pending_taken || remote_taken) && !travel_phase) slew_applied <= 32'sd0;
        else if (tick) begin

            if (advance_blocked) slew_applied <= crawl_slew;

            else if (slew_applied < slew_goal)
                slew_applied <= (accel_eff <= 0 || (slew_goal - slew_applied) <= accel_eff)
                              ? slew_goal : (slew_applied + accel_eff);
            else if (slew_applied > slew_goal)
                slew_applied <= ((slew_applied - slew_goal) <= decel_eff)
                              ? slew_goal : (slew_applied - decel_eff);
        end
    end

    reg [15:0] ready_count;
    reg [15:0] reached_count;
    reg [15:0] home_count;
    reg [15:0] key_still_count;
    wire key_settled = (KEY_SETTLE_TICKS==0) ||
                       ((dpos_abs==0) && (key_still_count>=KEY_SETTLE_TICKS-1));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) key_still_count<=0;
        else if(re_zero || !active || fault || !travel_phase || !key_leg || key_arrival)
            key_still_count<=0;
        else if(tick) begin
            if(target_ref!=target_cmd || follow_abs>SETTLE_ERROR_MAX || dpos_abs!=0)
                key_still_count<=0;
            else if(key_still_count<KEY_SETTLE_TICKS)
                key_still_count<=key_still_count+1'b1;
        end
    end
    wire signed [31:0] actual_q16 = {actual_pos,16'd0};
    wire signed [32:0] actual_ext = {actual_q16[31],actual_q16};
    wire home_in_range = (actual_ext<=limit_ext) && (actual_ext>=neg_limit_ext);

    assign homing = active && !fault && qualified && !ready && !home_fault;
    assign moving = active && !fault && qualified && !home_fault && (target_ref != target_cmd);
    assign paused = moving && advance_blocked;

    wire home_settled = (target_ref==target_cmd) &&
                        (follow_abs<=SETTLE_ERROR_MAX) && dpos_good && angle_good;
    assign at_target = (target_ref==target_cmd) && position_reached &&
                       dpos_good && angle_good;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_ref   <= 32'sd0;
            target_cmd   <= 32'sd0;
            ready         <= 1'b0;
            ready_count   <= 16'd0;
            reached       <= 1'b0;
            reached_count <= 16'd0;
            home_count    <= 16'd0;
            was_active    <= 1'b0;
            qualified     <= 1'b0;
            home_fault    <= 1'b0;
            current_slew  <= HOME_SLEW_Q16;
            travel_phase<=0; recovery_hold<=0; key_leg<=1'b0; leg_long<=1'b0;
            previous_angle<=0; angle_history_valid<=0;
            home_leg<=1'b0; key_arrival<=1'b0;
        end else if (re_zero) begin

            target_ref<=0; target_cmd<=0; ready<=0; ready_count<=0;
            reached<=0; reached_count<=0; home_count<=0; qualified<=0;
            was_active<=active; home_fault<=0;
            current_slew<=HOME_SLEW_Q16;
            travel_phase<=0; recovery_hold<=0; angle_history_valid<=0;
            leg_long<=1'b0;
            home_leg<=1'b0; key_arrival<=1'b0;
        end else if (!active || fault) begin

            target_ref   <= actual_q16;
            target_cmd   <= actual_q16;
            ready         <= 1'b0;
            ready_count   <= 16'd0;
            reached       <= 1'b0;
            reached_count <= 16'd0;
            home_count<=0; was_active<=0; qualified<=0; home_fault<=0;
            current_slew<=HOME_SLEW_Q16;
            travel_phase<=0; recovery_hold<=0; angle_history_valid<=0;
            leg_long<=1'b0;
            home_leg<=1'b0; key_arrival<=1'b0;
        end else if (!was_active) begin
            target_ref<=actual_q16; target_cmd<=actual_q16;
            was_active<=1; qualified<=0; ready<=0; ready_count<=0;
            reached<=0; reached_count<=0; home_count<=0;
            home_fault<=!home_in_range;
            current_slew<=HOME_SLEW_Q16;
            travel_phase<=0; recovery_hold<=0; angle_history_valid<=0;
            leg_long<=1'b0;
            home_leg<=1'b0; key_arrival<=1'b0;
        end else begin
            if (tick) begin
                previous_angle<=angle_error; angle_history_valid<=1;
                if (!traveling) recovery_hold<=0;
                else if (travel_unsafe) recovery_hold<=1;
                else if (recovery_good) recovery_hold<=0;

                if (traveling && target_ref==target_cmd &&
                    follow_abs<=SETTLE_ERROR_MAX && dpos_good && (!key_leg || key_settled)) begin
                    travel_phase<=0; recovery_hold<=0;

                    if (key_leg) key_arrival<=1'b1;
                end
            end
            if (tick && !home_fault) begin

                if (!balance_confirmed) begin
                    qualified<=0; ready<=0; ready_count<=0;
                    reached<=0; reached_count<=0; home_count<=0;
                    travel_phase<=0; recovery_hold<=0;
                    leg_long<=1'b0;
                    home_leg<=1'b0;
                end else if (!qualified) begin
                    reached       <= 1'b0;
                    reached_count <= 16'd0;

                    if (angle_good && dpos_good && qualify_pos) begin
                        if (ready_count >= qualify_need - 16'd1) begin
                            qualified   <= 1'b1;
                            target_cmd  <= 32'sd0;

                            current_slew <= home_slew_eff;

                            if (home_ramp_en) home_leg <= 1'b1;
                            ready_count <= ready_count;
                        end else begin
                            ready_count <= ready_count + 1'b1;
                            home_leg    <= 1'b0;
                        end
                    end else begin
                        ready_count <= 16'd0;
                        home_leg    <= 1'b0;
                    end
                end else begin

                    if (!advance_blocked && (slew_used > 0)) begin
                        if ($signed(target_ref) < $signed(target_cmd)) begin
                            if (distance_up <= slew_ext)
                                target_ref <= target_cmd;
                            else
                                target_ref <= target_ref + slew_used;
                        end else if ($signed(target_ref) > $signed(target_cmd)) begin
                            if (distance_down <= slew_ext)
                                target_ref <= target_cmd;
                            else
                                target_ref <= target_ref - slew_used;
                        end
                    end

                    if (home_leg && !actual_far && dpos_good)
                        home_leg <= 1'b0;

                    if (at_target || (!ready && home_settled)) begin
                        if (!reached) begin
                            if (reached_count >= REACHED_TICKS - 1) begin
                                reached       <= at_target;
                                ready         <= 1'b1;
                                reached_count <= reached_count;
                            end else begin
                                reached_count <= reached_count + 1'b1;
                            end
                        end
                    end else begin
                        reached       <= 1'b0;
                        reached_count <= 16'd0;
                    end
                    if (!ready) begin
                        if (home_count>=HOME_TIMEOUT_TICKS-1) begin
                            home_fault<=1; reached<=0; ready<=0;
                            home_leg<=1'b0;

                            target_ref<=target_ref;
                        end else home_count<=home_count+1'b1;
                    end
                end
            end

            if (!key_arrival && ready && balance_confirmed && !home_fault && eff_any && !leg_active_w) begin
                target_ref   <= target_ref;
                target_cmd   <= eff_command;
                current_slew <= slew_eff;
                travel_phase<=1; recovery_hold<=0; home_leg<=1'b0;
                key_leg<=1'b1;
                leg_long<=1'b0;

                reached       <= 1'b0;
                reached_count <= 16'd0;

                key_arrival   <= 1'b0;
            end else if (remote_taken) begin

                target_ref   <= remote_long ? 32'sd0 : target_ref;
                target_cmd   <= remote_command;
                current_slew <= slew_eff;
                travel_phase<=1; recovery_hold<=0; home_leg<=1'b0;
                key_leg<=1'b1;
                leg_long<=remote_long;
                reached<=0; reached_count<=0;
                key_arrival<=1'b0;
            end else if (!key_arrival && ready && balance_confirmed && !home_fault && hold_motion) begin

                target_ref<=target_ref; target_cmd<=target_ref;
                travel_phase<=0; recovery_hold<=0; home_leg<=1'b0;
                leg_long<=1'b0;
                reached<=0; reached_count<=0;
                key_arrival<=1'b0;
            end else if (!key_arrival && ready && balance_confirmed && !home_fault && profile_request &&
                         !leg_active_w) begin
                target_ref<=target_ref;
                target_cmd<=bounded_profile_target; current_slew<=bounded_profile_slew;
                travel_phase<=1; recovery_hold<=0; home_leg<=1'b0;
                key_leg<=1'b0;
                leg_long<=1'b0;
                reached<=0; reached_count<=0;
                key_arrival<=1'b0;
            end else if (key_arrival) begin

                target_ref   <= 32'sd0;
                target_cmd   <= 32'sd0;
                travel_phase <= 1'b0;
                key_leg      <= 1'b0;
                leg_long     <= 1'b0;
                home_leg     <= 1'b0;
                key_arrival  <= 1'b0;

                if (ready && balance_confirmed && !home_fault && profile_request && !eff_any) begin
                    target_cmd<=bounded_profile_target;current_slew<=bounded_profile_slew;
                    travel_phase<=1;recovery_hold<=0;reached<=0;reached_count<=0;
                    leg_long<=1'b0;
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending_key <= 2'd0; pending_age <= 16'd0;
        end else if (!active || !balance_confirmed || fault || home_fault) begin
            pending_key <= 2'd0; pending_age <= 16'd0;
        end else if (remote_move_req) begin

            pending_key <= 2'd0; pending_age <= 16'd0;
        end else if (key_any && (!ready || key_arrival || re_zero) && !home_fault && !fault) begin

            pending_key <= key_zero ? 2'd3 : (key_neg ? 2'd1 : 2'd2);
            pending_age <= PENDING_MS[15:0];
        end else if (pending_valid && pending_taken) begin
            pending_key <= 2'd0; pending_age <= 16'd0;
        end else if (pending_valid && tick && pending_age == 16'd0) begin
            pending_key <= 2'd0;
        end else if (pending_valid && tick) begin
            pending_age <= pending_age - 1'b1;
        end
    end

    wire signed [17:0] arrival_actual = leg_long ? actual_leg
                                                 : {{2{actual_pos[15]}}, actual_pos};
    wire signed [18:0] arrival_err_wide =
        {{1{arrival_actual[17]}}, arrival_actual} - {{3{target_cmd[31]}}, target_cmd[31:16]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arrival_error <= 16'sd0;
            arrival_count <= 8'd0;
        end else if (key_arrival) begin
            arrival_error <= (arrival_err_wide > 19'sd32767) ? 16'sh7fff :
                             (arrival_err_wide < -19'sd32768) ? 16'sh8000 :
                             arrival_err_wide[15:0];
            arrival_count <= arrival_count + 1'b1;
        end
    end

endmodule
