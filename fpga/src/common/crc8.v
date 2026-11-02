`timescale 1ns / 1ps

module crc8 (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       init,
    input  wire       en,
    input  wire [7:0] data_in,
    output reg  [7:0] crc
);

    integer   i;
    reg [7:0] c;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc <= 8'h00;
        end else if (init) begin
            crc <= 8'h00;
        end else if (en) begin

            c = crc ^ data_in;
            for (i = 0; i < 8; i = i + 1) begin
                if (c[7]) c = (c << 1) ^ 8'h07;
                else      c = (c << 1);
            end
            crc <= c;
        end
    end

endmodule
