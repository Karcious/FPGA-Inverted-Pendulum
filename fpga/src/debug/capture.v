`timescale 1ns / 1ps

module capture #(
    parameter integer DEPTH = 4096,
    parameter integer AW    = 12,
    parameter integer NCH   = 4,
    parameter integer DW    = 16,
    parameter integer DECIM = 1,
    parameter integer DCW   = 1,
    parameter integer RUNTIME_AVERAGE = 0
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  sample_en,
    input  wire [NCH*DW-1:0]     sample_data,
    input  wire [31:0]          average_log2,

    input  wire [1:0]            trig_ch,
    input  wire signed [DW-1:0]  trig_level,
    input  wire [1:0]            trig_edge,
    input  wire [AW-1:0]         pre_len,

    input  wire                  arm,
    input  wire                  disarm,

    output reg                   armed,
    output reg                   done,
    output reg  [AW-1:0]         trig_idx,

    input  wire [AW-1:0]         rd_addr,
    output reg  [NCH*DW-1:0]     rd_data
);

    reg [NCH*DW-1:0] mem [0:DEPTH-1];

    reg [2:0] avg_shift;
    wire [15:0] decim_m1 = RUNTIME_AVERAGE ? ((16'd1 << avg_shift)-1'b1) : DECIM-1;
    reg [15:0] dec_cnt;
    wire store_en = sample_en && (dec_cnt == decim_m1);
    wire [NCH*DW-1:0] store_data;
    genvar channel;
    generate for(channel=0;channel<NCH;channel=channel+1) begin: averages
        reg signed [DW+4:0] accum;
        wire signed [DW-1:0] value = sample_data[channel*DW +: DW];
        wire signed [DW+4:0] sum = accum + {{5{value[DW-1]}},value};
        wire signed [DW+4:0] mean = sum >>> avg_shift;
        assign store_data[channel*DW +: DW] = RUNTIME_AVERAGE ? mean[DW-1:0] : value;
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) accum<=0;
            else if(arm || disarm || !armed) accum<=0;
            else if(sample_en) accum <= store_en ? 0 : sum;
        end
    end endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin dec_cnt <= 0; avg_shift<=0; end
        else if (arm || disarm) begin
            dec_cnt <= 0;
            if(arm) avg_shift <= average_log2 > 4 ? 3'd4 : average_log2[2:0];
        end else if (sample_en && armed) begin
            if (dec_cnt == decim_m1) dec_cnt <= 0;
            else                     dec_cnt <= dec_cnt + 1'b1;
        end
    end

    reg [AW-1:0]       wr_ptr;
    reg                triggered;
    reg [AW:0]         remain;
    reg [AW-1:0]       rd_base;
    reg                primed;

    reg signed [DW-1:0] ch_prev;
    wire signed [DW-1:0] ch_cur = store_data[trig_ch*DW +: DW];

    wire rising  = (ch_prev <  trig_level) && (ch_cur >= trig_level);
    wire falling = (ch_prev >  trig_level) && (ch_cur <= trig_level);
    wire changed = (ch_cur  != ch_prev);

    reg [AW:0] filled;
    wire trig_hit = armed && !triggered && store_en && primed && (filled >= pre_len) &&
                    ((trig_edge == 2'd0) ? rising  :
                     (trig_edge == 2'd1) ? falling :
                     (trig_edge == 2'd2) ? changed : 1'b0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            armed     <= 1'b0;
            done      <= 1'b0;
            triggered <= 1'b0;
            wr_ptr    <= {AW{1'b0}};
            trig_idx  <= {AW{1'b0}};
            rd_base   <= {AW{1'b0}};
            remain    <= {(AW+1){1'b0}};
            ch_prev   <= {DW{1'b0}};
            primed    <= 1'b0;
            filled    <= 0;
        end else if (arm) begin
            armed     <= 1'b1;
            done      <= 1'b0;
            triggered <= 1'b0;
            wr_ptr    <= {AW{1'b0}};
            remain    <= {(AW+1){1'b0}};
            ch_prev   <= {DW{1'b0}};
            primed    <= 1'b0;
            filled    <= 0;
        end else if (disarm) begin
            armed     <= 1'b0;
            triggered <= 1'b0;
            primed    <= 1'b0;
        end else begin

            if (store_en && armed) begin
                mem[wr_ptr] <= store_data;
                if(filled < DEPTH) filled<=filled+1'b1;
                wr_ptr      <= wr_ptr + 1'b1;
                ch_prev     <= ch_cur;
                primed      <= 1'b1;

                if (trig_hit) begin
                    triggered <= 1'b1;
                    trig_idx  <= wr_ptr;

                    rd_base   <= wr_ptr - pre_len;

                    remain    <= DEPTH - pre_len - 1;
                    if(pre_len == DEPTH-1) begin
                        triggered<=0; armed<=0; done<=1;
                    end
                end else if (triggered) begin
                    if (remain == {(AW+1){1'b0}} + 1'b1) begin
                        triggered <= 1'b0;
                        armed     <= 1'b0;
                        done      <= 1'b1;
                    end
                    remain <= remain - 1'b1;
                end
            end
        end
    end

    wire [AW-1:0] rd_phys = (rd_addr + rd_base) & {AW{1'b1}};

    always @(posedge clk) begin
        rd_data <= mem[rd_phys];
    end

endmodule
