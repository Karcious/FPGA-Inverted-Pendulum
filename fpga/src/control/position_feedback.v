`timescale 1ns/1ps
module position_feedback #(
    parameter integer RANGE = 8191,
    parameter integer ORIGIN_INIT = 10000,
    parameter integer CPR = 1040,

    parameter integer HYST = 64,

    parameter integer LATCH_ON_ENTRY = 1
)(
    input wire clk, rst_n, active,

    input wire session,

    input wire re_zero,
    input wire signed [31:0] encoder,
    input wire signed [31:0] target,

    input wire leg_active,
    output reg signed [15:0] relative_pos,

    output reg signed [17:0] leg_pos,
    output reg signed [31:0] error_q16,
    output reg fault,

    output wire origin_latched
);
    localparam integer HALF = CPR / 2;
    localparam integer STEP = HALF + HYST;

    reg was_active;
    reg homed;
    reg signed [31:0] origin;

    wire signed [32:0] disp = {encoder[31], encoder} - {origin[31], origin};

    reg  signed [32:0] leg_base;
    reg                leg_active_q;

    wire signed [32:0] leg_base_now = (leg_active && !leg_active_q) ? disp : leg_base;
    wire signed [32:0] disp_leg     = leg_active ? (disp - leg_base_now) : disp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            leg_base     <= 33'sd0;
            leg_active_q <= 1'b0;
        end else begin
            leg_active_q <= leg_active;
            if (re_zero)                              leg_base <= 33'sd0;
            else if (leg_active && !leg_active_q)     leg_base <= disp;
        end
    end

    reg signed [32:0] disp_pipe;

    wire latch_fire = LATCH_ON_ENTRY && session && active && !was_active &&
                      !homed && !re_zero;
    assign origin_latched = latch_fire;

    wire signed [15:0] HALF_S = HALF[15:0];
    wire signed [15:0] disp_wrap =
        (disp >  HALF) ?  HALF_S :
        (disp < -HALF) ? -HALF_S : disp[15:0];

    localparam signed [48:0] HALF_Q  = HALF  * 49'sd65536;
    localparam signed [48:0] FOLD_Q  = CPR   * 49'sd65536;
    localparam signed [48:0] LIMIT_Q = RANGE * 49'sd65536;

    wire signed [48:0] error_raw  = {{17{target[31]}}, target} - {disp_pipe, 16'd0};
    wire signed [48:0] error_fold = (error_raw >  HALF_Q) ? error_raw - FOLD_Q :
                                    (error_raw < -HALF_Q) ? error_raw + FOLD_Q : error_raw;

    wire signed [48:0] error_wide = (error_fold >  HALF_Q) ?  HALF_Q :
                                    (error_fold < -HALF_Q) ? -HALF_Q : error_fold;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            was_active <= 1'b0; homed <= 1'b0; origin <= ORIGIN_INIT;
            disp_pipe <= 33'sd0;
            relative_pos <= 16'sd0; leg_pos <= 18'sd0; error_q16 <= 32'sd0; fault <= 1'b0;
        end else begin
            was_active <= active;
            if (re_zero) begin

                homed <= 1'b1; origin <= encoder;
                disp_pipe <= 33'sd0;
                relative_pos <= 16'sd0; leg_pos <= 18'sd0; error_q16 <= 32'sd0;

                if (!active) fault <= 1'b0;
            end else if (!session) begin

                homed <= 1'b0;
                disp_pipe <= disp_leg;
                relative_pos <= disp_wrap; leg_pos <= disp_leg[17:0];
                error_q16 <= 32'sd0; fault <= 1'b0;
            end else if (!active) begin

                disp_pipe <= disp_leg;
                relative_pos <= disp_wrap; leg_pos <= disp_leg[17:0];
                error_q16 <= 32'sd0; fault <= 1'b0;
            end else if (LATCH_ON_ENTRY && !was_active && !homed) begin

                homed <= 1'b1; origin <= encoder;
                disp_pipe <= 33'sd0;
                relative_pos <= 16'sd0; leg_pos <= 18'sd0; error_q16 <= 32'sd0; fault <= 1'b0;
            end else begin

                if (!leg_active) begin
                    if      (disp >  STEP) origin <= origin + CPR;
                    else if (disp < -STEP) origin <= origin - CPR;
                end
                disp_pipe <= disp_leg;
                relative_pos <= disp_wrap; leg_pos <= disp_leg[17:0];

                error_q16 <= error_wide[31:0];
                if ($signed(target) > LIMIT_Q || $signed(target) < -LIMIT_Q)
                    fault <= 1'b1;
            end
        end
    end
endmodule
