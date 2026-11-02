`timescale 1ns / 1ps

module safety_guard #(

    parameter [15:0] THETA_SPAN = 16'd800,

    parameter [15:0] REL_SPAN   = 16'd400,

    parameter [15:0] DPOS_MAX   = 16'd3,

    parameter [15:0] DPOS_REL   = 16'd1,

    parameter [15:0] DTHETA_MAX = 16'd1024
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,
    input  wire        run,
    input  wire [15:0] theta,
    input  wire [15:0] center,
    input  wire signed [15:0] dpos,
    output reg         safe
);

    localparam [2:0] REL_CNT = 3'd7;

    wire [15:0] dev = (theta >= center) ? (theta - center) : (center - theta);

    wire fell_now = (dev > THETA_SPAN);
    wire dev_ok   = (dev < REL_SPAN);

    wire [15:0] dp_u   = dpos;
    wire [15:0] dp_abs = dp_u[15] ? (16'd0 - dp_u) : dp_u;

    wire spin_now = (dp_abs > DPOS_MAX);
    wire dp_ok    = (dp_abs <= DPOS_REL);

    reg  [15:0] theta_prev;
    reg         primed;

    wire [15:0] dth = (theta >= theta_prev) ? (theta - theta_prev)
                                            : (theta_prev - theta);
    wire wrap_now = primed && (dth > DTHETA_MAX);

    reg [2:0] ok_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            safe       <= 1'b0;
            primed     <= 1'b0;
            theta_prev <= 16'd0;
            ok_cnt     <= 3'd0;
        end else if (en) begin
            primed     <= 1'b1;
            theta_prev <= theta;

            if (!run) begin

                safe   <= 1'b0;
                ok_cnt <= 3'd0;
            end else if (fell_now || spin_now || wrap_now) begin
                safe   <= 1'b1;
                ok_cnt <= 3'd0;
            end else if (safe) begin
                if (ok_cnt == REL_CNT && dev_ok && dp_ok) begin
                    safe   <= 1'b0;
                    ok_cnt <= 3'd0;
                end else if (dev_ok && dp_ok) begin
                    ok_cnt <= ok_cnt + 1'b1;
                end else begin
                    ok_cnt <= 3'd0;
                end
            end
        end
    end

endmodule
