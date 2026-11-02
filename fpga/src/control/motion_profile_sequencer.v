`timescale 1ns/1ps

module motion_profile_sequencer #(
    parameter integer SEGMENT_TIMEOUT_MS=60000,
    parameter signed [31:0] LIMIT_Q16=32'sd34078720,
    parameter signed [31:0] MAX_SLEW_Q16=32'sd3787
)(
    input wire clk, rst_n, tick,
    input wire enable, cancel, re_zero,
    input wire start_a, start_b,
    input wire reached, at_target,
    output wire command_valid,
    output wire signed [31:0] command_target,
    output wire signed [31:0] command_slew,
    output reg hold_motion,
    output wire busy,
    output wire paused,
    output reg done,
    output reg fault,
    output reg profile_id,
    output reg [2:0] segment
);
    localparam [1:0] IDLE=0, ISSUE=1, WAIT_REACHED=2, DWELL=3;
    reg [1:0] state;
    reg [15:0] elapsed_ms, dwell_count;
    wire [15:0] dwell_ms;
    wire last, valid;
    motion_profile_rom u_rom(
        .profile_id(profile_id),.segment(segment),
        .target_q16(command_target),.slew_q16(command_slew),
        .dwell_ms(dwell_ms),.last(last),.valid(valid));

    wire signed [32:0] target_ext={command_target[31],command_target};
    wire signed [32:0] limit_ext={LIMIT_Q16[31],LIMIT_Q16};
    wire entry_valid=valid && target_ext<=limit_ext && target_ext>=-limit_ext &&
                     command_slew>0 && command_slew<=MAX_SLEW_Q16;
    assign busy=(state!=IDLE) && enable && !cancel && !re_zero && !fault;
    assign command_valid=(state==ISSUE) && busy && entry_valid;
    assign paused=busy && (state==DWELL) && !(reached && at_target);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; elapsed_ms<=0; dwell_count<=0; hold_motion<=0;
            done<=0; fault<=0; profile_id<=0; segment<=0;
        end else begin
            hold_motion<=0;
            if (!enable || cancel || re_zero) begin
                state<=IDLE; elapsed_ms<=0; dwell_count<=0;
                done<=0; fault<=0; profile_id<=0; segment<=0;
            end else if (state==IDLE) begin

                if (!fault && (start_a ^ start_b)) begin
                    profile_id<=start_b; segment<=0; state<=ISSUE;
                    done<=0; elapsed_ms<=0; dwell_count<=0;
                end
            end else if (state==ISSUE) begin
                elapsed_ms<=0; dwell_count<=0;
                if (entry_valid) state<=WAIT_REACHED;
                else begin state<=IDLE; fault<=1; hold_motion<=1; end
            end else if (tick) begin

                if (elapsed_ms>=SEGMENT_TIMEOUT_MS-1) begin
                    state<=IDLE; fault<=1; done<=0; hold_motion<=1;
                end else begin
                    elapsed_ms<=elapsed_ms+1'b1;
                    if (state==WAIT_REACHED && reached && at_target) begin
                        if (dwell_ms!=0) begin state<=DWELL; dwell_count<=0; end
                        else if (last) begin state<=IDLE; done<=1; end
                        else begin segment<=segment+1'b1; state<=ISSUE; end
                    end else if (state==DWELL && reached && at_target) begin
                        if (dwell_count>=dwell_ms-1) begin
                            if (last) begin state<=IDLE; done<=1; end
                            else begin segment<=segment+1'b1; state<=ISSUE; end
                        end else dwell_count<=dwell_count+1'b1;
                    end
                end
            end
        end
    end
endmodule
