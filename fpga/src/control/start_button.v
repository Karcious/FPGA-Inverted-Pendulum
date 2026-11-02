`timescale 1ns/1ps

module start_button #(
    parameter integer DEBOUNCE_MS=20,
    parameter integer LONG_PRESS_MS=0
)(
    input wire clk, rst_n, tick, sw1_n,
    output reg press,
    output reg long_press
);
    reg sync1, sync2, stable, armed;
    reg [15:0] count;
    reg pending_short;
    reg [15:0] hold_count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync1<=0; sync2<=0; stable<=0; armed<=0; count<=0; press<=0;
            long_press<=0; pending_short<=0; hold_count<=0;
        end else begin
            sync1<=sw1_n; sync2<=sync1; press<=0; long_press<=0;
            if (tick) begin

                if (LONG_PRESS_MS>0 && pending_short && !stable && !sync2) begin
                    if (hold_count>=LONG_PRESS_MS-1) begin
                        long_press<=1; pending_short<=0;
                    end else hold_count<=hold_count+1'b1;
                end
                if (sync2==stable) count<=0;
                else if (count==DEBOUNCE_MS-1) begin
                    count<=0; stable<=sync2;
                    if (sync2) begin
                        armed<=1;
                        if (LONG_PRESS_MS>0 && pending_short) press<=1;
                        pending_short<=0; hold_count<=0;
                    end else if (armed) begin
                        armed<=0; hold_count<=0;
                        if (LONG_PRESS_MS>0) pending_short<=1;
                        else press<=1;
                    end
                end else count<=count+1'b1;
            end
        end
    end
endmodule
