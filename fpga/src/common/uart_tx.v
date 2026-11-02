`timescale 1ns / 1ps

module uart_tx #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD     = 921600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       send,
    input  wire [7:0] data_in,
    output reg        tx,
    output reg        busy
);

    localparam integer DIV = CLK_FREQ / BAUD;

    reg [9:0]  shifter;
    reg [15:0] cnt;
    reg [3:0]  bit_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx      <= 1'b1;
            busy    <= 1'b0;
            cnt     <= 16'd0;
            bit_idx <= 4'd0;
            shifter <= 10'h3FF;
        end else if (!busy) begin
            tx <= 1'b1;
            if (send) begin
                shifter <= {1'b1, data_in, 1'b0};
                busy    <= 1'b1;
                bit_idx <= 4'd0;
                cnt     <= DIV - 1;
                tx      <= 1'b0;
            end
        end else begin
            if (cnt != 16'd0) begin
                cnt <= cnt - 1'b1;
            end else begin
                cnt <= DIV - 1;
                if (bit_idx == 4'd9) begin
                    busy    <= 1'b0;
                    bit_idx <= 4'd0;
                    tx      <= 1'b1;
                end else begin
                    bit_idx <= bit_idx + 1'b1;
                    tx      <= shifter[bit_idx + 1'b1];
                end
            end
        end
    end

endmodule
