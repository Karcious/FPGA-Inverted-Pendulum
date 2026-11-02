`timescale 1ns / 1ps

module adc_capture #(

    parameter integer OSR = 256,

    parameter integer SHR = 6
)(
    input  wire        clk,
    input  wire        rst_n,

    output reg         adc_clk,
    output wire        adc_oe_n,
    input  wire [9:0]  adc_data_in,
    input  wire        adc_otr,

    output reg  [11:0] theta_raw,
    output reg         adc_valid,
    output reg         otr_seen
);

    always @(posedge clk or negedge rst_n)
        if (!rst_n) adc_clk <= 1'b0;
        else        adc_clk <= ~adc_clk;

    assign adc_oe_n = 1'b0;

    reg [9:0] adc_s;
    reg       adc_s_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_s   <= 10'd0;
            adc_s_v <= 1'b0;
        end else begin
            adc_s_v <= ~adc_clk;
            if (!adc_clk) adc_s <= adc_data_in;
        end
    end

    reg [18:0] acc_sum;
    reg [7:0]  acc_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_sum   <= 19'd0;
            acc_cnt   <= 8'd0;
            theta_raw <= 12'd0;
            adc_valid <= 1'b0;
        end else begin
            adc_valid <= 1'b0;
            if (adc_s_v) begin
                if (acc_cnt == OSR - 1) begin

                    theta_raw <= (acc_sum + {9'd0, adc_s}) >> SHR;
                    adc_valid <= 1'b1;
                    acc_sum   <= 19'd0;
                    acc_cnt   <= 8'd0;
                end else begin
                    acc_sum <= acc_sum + {9'd0, adc_s};
                    acc_cnt <= acc_cnt + 1'b1;
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n)
        if (!rst_n)       otr_seen <= 1'b0;
        else if (adc_otr) otr_seen <= 1'b1;

endmodule

module adc_angle_filter (
    input wire clk, rst_n,
    input wire sample_valid,
    input wire [11:0] sample,
    output reg [11:0] filtered
);
    reg [11:0] x1, x2, median_q;
    reg history_ready, median_valid, filter_ready;
    wire [11:0] median_value = (sample > x1) ?
        ((x1 > x2) ? x1 : ((sample > x2) ? x2 : sample)) :
        ((sample > x2) ? sample : ((x1 > x2) ? x2 : x1));

    reg signed [20:0] state_q8;
    wire signed [20:0] input_q8 = $signed({1'b0, median_q, 8'b0});
    wire signed [20:0] difference = input_q8 - state_q8;
    wire signed [20:0] rounded = state_q8 + 21'sd128;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x1 <= 0; x2 <= 0; median_q <= 0;
            history_ready <= 0; median_valid <= 0; filter_ready <= 0;
            state_q8 <= 0; filtered <= 0;
        end else begin
            median_valid <= sample_valid;
            if (sample_valid) begin
                if (!history_ready) begin
                    x1 <= sample; x2 <= sample; median_q <= sample;
                    history_ready <= 1;
                end else begin
                    x2 <= x1; x1 <= sample; median_q <= median_value;
                end
            end
            if (median_valid) begin

                if (!filter_ready || difference > 21'sd524288 ||
                                     difference < -21'sd524288)
                    state_q8 <= input_q8;
                else
                    state_q8 <= state_q8 + (difference >>> 5);
                filter_ready <= 1;
            end
            if (filter_ready) filtered <= rounded[19:8];
        end
    end
endmodule
