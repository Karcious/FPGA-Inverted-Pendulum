`timescale 1ns / 1ps

module uart_rx #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD     = 921600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid
);

    localparam integer DIV  = CLK_FREQ / BAUD;
    localparam integer HALF = (DIV > 1) ? (DIV / 2) : 1;

    localparam [2:0] S_IDLE  = 3'd0,
                     S_START = 3'd1,
                     S_DATA  = 3'd2,
                     S_STOP  = 3'd3;

    reg [1:0]  rx_sync;
    reg [2:0]  state;
    reg [15:0] cnt;
    reg [3:0]  bit_idx;
    reg [7:0]  shifter;

    wire rx_bit = rx_sync[1];

    always @(posedge clk) begin
        rx_sync <= {rx_sync[0], rx};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            cnt     <= 16'd0;
            bit_idx <= 4'd0;
            shifter <= 8'h00;
            data    <= 8'h00;
            valid   <= 1'b0;
        end else begin
            valid <= 1'b0;

            case (state)

                S_IDLE: begin
                    if (!rx_bit) begin
                        cnt   <= HALF - 1;
                        state <= S_START;
                    end
                end

                S_START: begin
                    if (cnt == 16'd0) begin
                        if (!rx_bit) begin
                            cnt     <= DIV - 1;
                            bit_idx <= 4'd0;
                            state   <= S_DATA;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else begin
                        cnt <= cnt - 1'b1;
                    end
                end

                S_DATA: begin
                    if (cnt == 16'd0) begin
                        shifter <= {rx_bit, shifter[7:1]};
                        cnt     <= DIV - 1;
                        if (bit_idx == 4'd7) state <= S_STOP;
                        else                 bit_idx <= bit_idx + 1'b1;
                    end else begin
                        cnt <= cnt - 1'b1;
                    end
                end

                S_STOP: begin
                    if (cnt == 16'd0) begin
                        data  <= shifter;
                        valid <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        cnt <= cnt - 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
