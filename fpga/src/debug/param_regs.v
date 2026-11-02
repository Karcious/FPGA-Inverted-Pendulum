`timescale 1ns / 1ps

module param_regs #(

    parameter integer N_PARAM = 52,
    parameter integer PW      = 6,

    parameter [31:0]  MAGIC   = 32'h0002_0719,
    parameter integer STATUS_ENABLE = 0,
    parameter integer BUTTON_ENABLE = 0
)(
    input  wire              button_press,
    input  wire              clk,
    input  wire              rst_n,
    input  wire [31:0]       control_status,
    input wire [895:0] trajectory_readouts,

    input  wire              wr_en,
    input  wire [PW-1:0]     wr_addr,
    input  wire [31:0]       wr_data,

    input  wire [PW-1:0]     rd_addr,
    output reg  [31:0]       rd_data,

    output wire [N_PARAM*32-1:0] param_flat
);

    reg [31:0] mem [0:N_PARAM-1];

    genvar g;
    generate
        for (g = 0; g < N_PARAM; g = g + 1) begin : gen_flat
            assign param_flat[g*32 +: 32] = (STATUS_ENABLE && g==31) ? control_status : mem[g];
        end
    endgenerate

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < N_PARAM; k = k + 1) mem[k] <= 32'h0000_0000;

            mem[ 0] <= MAGIC;
            mem[ 1] <= 32'd1000;
            mem[ 2] <= 32'd20000;

            mem[ 3] <= 32'h0004_8000;

            mem[ 4] <= 32'h0078_0000;

            mem[ 5] <= 32'hFFFE_CCCD;

            mem[ 6] <= 32'hFFFF_FC29;

            mem[ 7] <= 32'd30;

            mem[ 8] <= 32'd600;

            mem[ 9] <= 32'd271;
            mem[10] <= 32'd130;

            mem[11] <= 32'd1024;
            mem[12] <= 32'h0004_0000;
            mem[13] <= 32'h0000_0000;
            mem[14] <= 32'h0001_999a;

            mem[15] <= 32'd135;

            mem[18] <= 32'd3075;

            mem[19] <= 32'd2;

            mem[20] <= 32'hFFD8_0000;
            mem[23] <= 32'd50;

            mem[24] <= -32'sd900;
            mem[25] <= 32'd600;
            mem[26] <= 32'd1500;

            mem[27] <= 32'd520;
            mem[28] <= 32'd1017;

            mem[29] <= 32'd350;

            mem[30] <= 32'd55;

            mem[32] <= 32'd0;

            mem[33] <= 32'hFFFE_CCCD;
            mem[34] <= 32'hFFD8_0000;
            mem[35] <= 32'hFFFE_CCCD;
            mem[36] <= 32'hFFD8_0000;
            mem[37] <= 32'd0;
            mem[38] <= 32'd0;

            mem[39] <= 32'h0002_0000;

            mem[40] <= 32'd13107;
            mem[41] <= 32'd3787;
            mem[42] <= 32'd98;
            mem[43] <= 32'd0;

            mem[44] <= 32'hFFF5_0000;
            mem[45] <= 32'd0;
            mem[46] <= 32'd4;

            mem[47] <= 32'h0000_4000;

            mem[48] <= 32'd0;

            mem[49] <= 32'd30;

            mem[50] <= 32'd1;

            mem[51] <= 32'd0;
        end else begin

            if (BUTTON_ENABLE && button_press)
                mem[16] <= (mem[16]==0) ? 32'd3 : 32'd0;
            if (wr_en && (wr_addr != {PW{1'b0}}) && wr_addr<N_PARAM && !(STATUS_ENABLE && wr_addr==31)) begin
                mem[wr_addr] <= wr_data;
            end
        end
    end

    always @(*) begin
        rd_data = (PW>=8 && rd_addr>=64 && rd_addr<92) ? trajectory_readouts[(rd_addr-64)*32+:32] :
                  (STATUS_ENABLE && rd_addr==31) ? control_status :
                  (rd_addr < N_PARAM) ? mem[rd_addr] : 32'hDEAD_BEEF;
    end

endmodule
