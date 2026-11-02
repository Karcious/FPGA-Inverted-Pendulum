`timescale 1ns / 1ps

`include "dbg_pkg.vh"

module dbg_top #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD     = 921600,
    parameter integer NCH      = 4,
    parameter integer DW       = 16,
    parameter integer DEPTH    = 4096,
    parameter integer AW       = 12,
    parameter integer DECIM    = 1,
    parameter integer DCW      = 1,

    parameter integer N_PARAM  = 52,
    parameter integer PW       = 6,

    parameter integer BUTTON_ENABLE = 0,

    parameter [15:0]  VER      = 16'h0207,

    parameter [7:0]   REV      = 8'h19
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire button_press,
    input  wire [31:0]           control_status,
    input wire [895:0] trajectory_readouts,
    output wire trajectory_wr_en,
    output wire [7:0] trajectory_wr_addr,
    output wire [31:0] trajectory_wr_data,
    input  wire [127:0]          fuzzy_debug_data,

    input  wire                  uart_rx_pin,
    output wire                  uart_tx_pin,

    output wire [N_PARAM*32-1:0] param_flat,

    input  wire                  sample_en,

    input  wire [NCH*DW-1:0]     probe_data,

    output wire                  cap_armed,
    output wire                  cap_done
);

    wire                  cap_arm;
    wire                  cap_disarm;
    wire [1:0]            cap_trig_ch;
    wire signed [DW-1:0]  cap_trig_level;
    wire [1:0]            cap_trig_edge;
    wire [AW-1:0]         cap_pre_len;
    wire                  cap_armed_i;
    wire                  cap_done_i;
    wire [AW-1:0]         cap_trig_idx;
    wire [AW-1:0]         cap_rd_addr;
    wire [NCH*DW-1:0]     cap_rd_data;
    reg [31:0] live_sequence;
    reg [NCH*DW+63:0] live_snapshot;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            live_sequence <= 0;
            live_snapshot <= 0;
        end else if (sample_en) begin
            live_sequence <= live_sequence + 1'b1;
            live_snapshot <= {control_status, live_sequence + 32'd1, probe_data};
        end
    end

    wire              p_wr_en;
    wire [PW-1:0]     p_wr_addr;
    wire [31:0]       p_wr_data;
    assign trajectory_wr_en=p_wr_en;
    assign trajectory_wr_addr=p_wr_addr;
    assign trajectory_wr_data=p_wr_data;
    wire [PW-1:0]     p_rd_addr;
    wire [31:0]       p_rd_data;

    dbg_proto #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD),
        .N_PARAM  (N_PARAM),
        .PW       (PW),
        .DEPTH    (DEPTH),
        .AW       (AW),
        .NCH      (NCH),
        .DW       (DW),
        .VER      (VER),
        .REV      (REV),
        .BUTTON_ENABLE(BUTTON_ENABLE)
    ) u_proto (
        .button_press(button_press),
        .control_status(control_status),
        .trajectory_readouts(trajectory_readouts),
        .live_data(live_snapshot),
        .fuzzy_data(fuzzy_debug_data),
        .clk            (clk),
        .rst_n          (rst_n),
        .rx_pin         (uart_rx_pin),
        .tx_pin         (uart_tx_pin),
        .p_wr_en        (p_wr_en),
        .p_wr_addr      (p_wr_addr),
        .p_wr_data      (p_wr_data),
        .p_rd_addr      (p_rd_addr),
        .p_rd_data      (p_rd_data),
        .param_flat     (param_flat),
        .cap_arm        (cap_arm),
        .cap_disarm     (cap_disarm),
        .cap_trig_ch    (cap_trig_ch),
        .cap_trig_level (cap_trig_level),
        .cap_trig_edge  (cap_trig_edge),
        .cap_pre_len    (cap_pre_len),
        .cap_armed      (cap_armed_i),
        .cap_done       (cap_done_i),
        .cap_trig_idx   (cap_trig_idx),
        .cap_rd_addr    (cap_rd_addr),
        .cap_rd_data    (cap_rd_data)
    );

    capture #(
        .DEPTH (DEPTH),
        .AW    (AW),
        .NCH   (NCH),
        .DW    (DW),
        .DECIM (DECIM),
        .DCW   (DCW), .RUNTIME_AVERAGE(1)
    ) u_capture (
        .clk         (clk),
        .rst_n       (rst_n),
        .sample_en   (sample_en),
        .sample_data (probe_data),
        .average_log2(param_flat[`P_CAPTURE_LOG2*32 +: 32]),
        .trig_ch     (cap_trig_ch),
        .trig_level  (cap_trig_level),
        .trig_edge   (cap_trig_edge),
        .pre_len     (cap_pre_len),
        .arm         (cap_arm),
        .disarm      (cap_disarm),
        .armed       (cap_armed_i),
        .done        (cap_done_i),
        .trig_idx    (cap_trig_idx),
        .rd_addr     (cap_rd_addr),
        .rd_data     (cap_rd_data)
    );

    assign cap_armed = cap_armed_i;
    assign cap_done  = cap_done_i;

endmodule
