`timescale 1ns / 1ps

module encoder_decode #(
    parameter integer CW       = 32,
    parameter integer POS_INIT = 10000
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  enc_a,
    input  wire                  enc_b,

    input  wire                  sample_en,

    output reg  signed [CW-1:0]  pos,
    output reg  signed [15:0]    delta,
    output reg                   dir,

    output reg                   fault_tick,
    output reg                   fault_seen
);

    reg [1:0] sync_a, sync_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_a <= 2'd0;
            sync_b <= 2'd0;
        end else begin
            sync_a <= {sync_a[0], enc_a};
            sync_b <= {sync_b[0], enc_b};
        end
    end

    wire [1:0] st_now = {sync_a[1], sync_b[1]};

    reg [1:0] warm;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                  warm <= 2'd0;
        else if (warm != 2'd3)       warm <= warm + 1'b1;
    end
    wire en = (warm == 2'd3);

    reg [1:0] st_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) st_prev <= 2'd0;
        else        st_prev <= st_now;
    end

    wire [3:0] tr = {st_prev, st_now};

    wire up = (tr == 4'b00_10) || (tr == 4'b10_11) ||
              (tr == 4'b11_01) || (tr == 4'b01_00);

    wire dn = (tr == 4'b00_01) || (tr == 4'b01_11) ||
              (tr == 4'b11_10) || (tr == 4'b10_00);

    wire bad = (tr == 4'b00_11) || (tr == 4'b11_00) ||
               (tr == 4'b01_10) || (tr == 4'b10_01);

    wire signed [1:0] step = up ? 2'sd1 : (dn ? 2'd3 : 2'sd0);

    reg signed [15:0]  cnt;
    wire               cnt_en  = en && (up || dn);
    wire signed [15:0] cnt_nxt = cnt + step;

    reg               fpend;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin

            pos        <= POS_INIT;
            delta      <= 16'sd0;
            cnt        <= 16'sd0;
            dir        <= 1'b0;
            fpend      <= 1'b0;
            fault_tick <= 1'b0;
            fault_seen <= 1'b0;
        end else begin
            if (cnt_en) pos <= pos + step;
            if (up)     dir <= 1'b1;
            if (dn)     dir <= 1'b0;
            if (en && bad) fpend <= 1'b1;

            if (sample_en) begin
                delta      <= cnt_en ? cnt_nxt : cnt;
                cnt        <= 16'sd0;
                fault_tick <= fpend;
                fault_seen <= fault_seen | fpend;
                fpend      <= 1'b0;
            end else if (cnt_en) begin
                cnt <= cnt_nxt;
            end
        end
    end

endmodule
