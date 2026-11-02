`timescale 1ns / 1ps

module pwm_gen #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer PWM_HZ   = 20_000,
    parameter integer DSH      = 10,
    parameter integer COMP_SHAPING = 0
)(
    input  wire               clk,
    input  wire               rst_n,

    input  wire signed [15:0] duty_in,

    input  wire        [15:0] duty_max,
    input  wire        [15:0] duty_min,
    input  wire        [3:0] comp_shift,

    output reg                pwm,
    output reg                an1,
    output reg                an2,

    output reg  signed [15:0] duty_applied,
    output reg                blanking
);

    localparam integer PWM_PERIOD = CLK_FREQ / PWM_HZ;
    localparam integer PERIOD_M1  = PWM_PERIOD - 1;

    wire signed [16:0] d17     = {duty_in[15], duty_in};
    wire signed [16:0] mag_raw = d17[16] ? -d17 : d17;

    wire [16:0] full = 17'd1 <<< DSH;

    wire [16:0] lim = (duty_max == 16'd0) ? full : {1'b0, duty_max};

    wire [16:0] dmin = ({1'b0, duty_min} > lim) ? lim : {1'b0, duty_min};

    reg  [16:0] lim_q;
    reg  [16:0] dmin_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lim_q  <= full;
            dmin_q <= 17'd0;
        end else begin
            lim_q  <= lim;
            dmin_q <= dmin;
        end
    end

    wire [16:0] mag_lim = (mag_raw > lim_q) ? lim_q : mag_raw;

    reg  [16:0] mag_lim_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) mag_lim_q <= 17'd0;
        else        mag_lim_q <= mag_lim;
    end

    wire [3:0] shape_shift = COMP_SHAPING ? ((comp_shift > 3) ? 4'd3 : comp_shift) : 4'd0;

    reg [3:0] shape_shift_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) shape_shift_q<=0;
        else shape_shift_q<=shape_shift;
    end
    wire [19:0] shaped = {3'd0,mag_lim_q} << shape_shift_q;
    wire [16:0] comp = shaped > {3'd0,dmin_q} ? dmin_q : shaped[16:0];

    reg [16:0] comp_q, mag_base_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin comp_q<=0; mag_base_q<=0; end
        else begin comp_q<=comp; mag_base_q<=mag_lim_q; end
    end
    wire [17:0] mag_dz_sum = {1'b0, mag_base_q} + {1'b0, comp_q};

    wire [16:0] mag_dz = (mag_base_q == 17'd0)         ? 17'd0 :
                         (mag_dz_sum > {1'b0, lim_q}) ? lim_q :
                                                        mag_dz_sum[16:0];

    reg  [16:0] d17_q;
    reg  [16:0] d17_q2;
    reg  [16:0] d17_q3;
    reg  [16:0] mag_dz_q;

    reg  [15:0] cmp;

    wire [31:0] cmp_raw = (mag_dz_q * PWM_PERIOD) >>> DSH;
    wire [15:0] cmp_c   = (cmp_raw > PWM_PERIOD) ? PWM_PERIOD[15:0] : cmp_raw[15:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d17_q    <= 17'sd0;
            d17_q2   <= 17'sd0;
            d17_q3   <= 17'sd0;
            mag_dz_q <= 17'd0;
            cmp      <= 16'd0;
        end else begin
            d17_q    <= d17;
            d17_q2   <= d17_q;
            d17_q3   <= d17_q2;
            mag_dz_q <= mag_dz;
            cmp      <= cmp_c;
        end
    end

    wire active  = (d17 != 17'sd0);
    wire want_fw = (d17[16] == 1'b0);

    reg               dir_now;
    reg               blank;
    reg [15:0]        bcnt;

    wire flip_now = active && !blank && (want_fw != dir_now);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dir_now <= 1'b1;
            blank   <= 1'b0;
            bcnt    <= 16'd0;
        end else if (flip_now) begin

            blank   <= 1'b1;
            bcnt    <= PWM_PERIOD[15:0];
            dir_now <= want_fw;
        end else if (blank) begin
            if (bcnt <= 16'd1) blank <= 1'b0;
            bcnt <= bcnt - 1'b1;
        end
    end

    reg [15:0] pcnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                    pcnt <= 16'd0;
        else if (pcnt >= PERIOD_M1)    pcnt <= 16'd0;
        else                           pcnt <= pcnt + 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm  <= 1'b0;
            an1  <= 1'b0;
            an2  <= 1'b0;
            blanking <= 1'b0;
        end else begin

            blanking <= blank || flip_now;

            if (!active || blank || flip_now) begin
                pwm <= 1'b0;
                an1 <= 1'b0;
                an2 <= 1'b0;
            end else begin
                pwm <= (pcnt < cmp);
                an1 <=  dir_now;
                an2 <= ~dir_now;
            end
        end
    end

    wire signed [16:0] d_app = d17_q3[16] ? -{1'b0, mag_dz_q} : {1'b0, mag_dz_q};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) duty_applied <= 16'sd0;
        else if (!active || blank || flip_now) duty_applied <= 16'sd0;
        else        duty_applied <= d_app[15:0];
    end

endmodule
