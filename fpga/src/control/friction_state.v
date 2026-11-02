`timescale 1ns/1ps
module friction_state(
    input wire clk, rst_n, tick, active, enable,
    input wire signed [15:0] delta,
    input wire [15:0] static_comp, moving_comp,

    input wire [3:0] shape_sel,
    output wire [15:0] compensation,
    output wire [3:0] comp_shift,
    output reg moving
);

    localparam [15:0] RAMP_STEP = 16'd8;

    reg [4:0] count;
    reg [20:0] travel;
    reg [1:0] quiet;
    reg [15:0] level;
    wire [15:0] magnitude = delta[15] ? -delta : delta;
    wire [20:0] total = travel + {5'd0,magnitude};
    wire [15:0] dynamic_limit = moving_comp > static_comp ? static_comp : moving_comp;
    wire [15:0] desired = moving ? dynamic_limit : static_comp;
    assign compensation = enable ? level : static_comp;

    wire [3:0] shape_base = (shape_sel > 4'd3) ? 4'd3 : shape_sel;
    wire [3:0] shape_plus = (shape_base == 4'd3) ? 4'd3 : shape_base + 4'd1;
    assign comp_shift = enable ? shape_plus : shape_base;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count<=0; travel<=0; quiet<=0; level<=0; moving<=0;
        end else if (!active || !enable) begin
            count<=0; travel<=0; quiet<=0; level<=static_comp; moving<=0;
        end else if (tick) begin
            if (level < desired)
                level <= desired-level > RAMP_STEP ? level+RAMP_STEP : desired;
            else if (level > desired)
                level <= level-desired > RAMP_STEP ? level-RAMP_STEP : desired;
            if (count == 19) begin
                count<=0; travel<=0;
                if (total >= 3) begin moving<=1; quiet<=0; end
                else if (total <= 1) begin
                    if (quiet == 2) begin moving<=0; quiet<=0; end
                    else quiet<=quiet+1'b1;
                end else quiet<=0;
            end else begin count<=count+1'b1; travel<=total; end
        end
    end
endmodule
