`timescale 1ns / 1ps

`include "dbg_pkg.vh"

module dbg_proto #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD     = 921600,

    parameter integer N_PARAM  = 52,
    parameter integer PW       = 6,
    parameter integer DEPTH    = 1024,
    parameter integer AW       = 10,
    parameter integer NCH      = 4,
    parameter integer DW       = 16,
    parameter integer BUTTON_ENABLE = 0,

    parameter [15:0]  VER      = 16'h0207,
    parameter [7:0]   REV      = 8'h0E
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire button_press,
    input  wire [31:0]           control_status,
    input wire [895:0] trajectory_readouts,
    input  wire [NCH*DW+63:0]    live_data,
    input  wire [127:0]          fuzzy_data,

    input  wire                  rx_pin,
    output wire                  tx_pin,

    output reg                   p_wr_en,
    output reg  [PW-1:0]         p_wr_addr,
    output reg  [31:0]           p_wr_data,
    output wire [PW-1:0]         p_rd_addr,
    input  wire [31:0]           p_rd_data,
    output wire [N_PARAM*32-1:0] param_flat,

    output reg                   cap_arm,
    output reg                   cap_disarm,
    output reg  [1:0]            cap_trig_ch,
    output reg  signed [DW-1:0]  cap_trig_level,
    output reg  [1:0]            cap_trig_edge,
    output reg  [AW-1:0]         cap_pre_len,
    input  wire                  cap_armed,
    input  wire                  cap_done,
    input  wire [AW-1:0]         cap_trig_idx,
    output wire [AW-1:0]         cap_rd_addr,
    input  wire [NCH*DW-1:0]     cap_rd_data
);

    localparam integer CB = NCH * DW / 8;

    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(rx_pin),
        .data(rx_data), .valid(rx_valid)
    );

    reg  [7:0] tx_data;
    reg        tx_send;
    wire       tx_busy;

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .send(tx_send), .data_in(tx_data),
        .tx(tx_pin), .busy(tx_busy)
    );

    reg        crc_init;
    reg        crc_en;
    wire [7:0] crc_val;

    crc8 u_crc (
        .clk(clk), .rst_n(rst_n),
        .init(crc_init), .en(crc_en), .data_in(rx_data),
        .crc(crc_val)
    );

    param_regs #(.N_PARAM(N_PARAM), .PW(PW), .MAGIC({8'd0, VER, REV}), .STATUS_ENABLE(VER>=16'h0113), .BUTTON_ENABLE(BUTTON_ENABLE)) u_params (
        .clk(clk), .rst_n(rst_n),
        .button_press(button_press),
        .control_status(control_status),
        .trajectory_readouts(trajectory_readouts),
        .wr_en(p_wr_en), .wr_addr(p_wr_addr), .wr_data(p_wr_data),
        .rd_addr(p_rd_addr), .rd_data(p_rd_data),
        .param_flat(param_flat)
    );

    function [7:0] crc8_step;
        input [7:0] c_in;
        input [7:0] d;
        integer   b;
        reg [7:0] c;
        begin
            c = c_in ^ d;
            for (b = 0; b < 8; b = b + 1) begin
                if (c[7]) c = (c << 1) ^ 8'h07;
                else      c = (c << 1);
            end
            crc8_step = c;
        end
    endfunction

    localparam [2:0] RX_MAGIC0 = 3'd0,
                     RX_MAGIC1 = 3'd1,
                     RX_CMD    = 3'd2,
                     RX_LEN_L  = 3'd3,
                     RX_LEN_H  = 3'd4,
                     RX_PAY    = 3'd5,
                     RX_CRC    = 3'd6,
                     RX_EXEC   = 3'd7;

    reg [2:0]  rx_state;
    reg [7:0]  rx_cmd;
    reg [15:0] rx_len;
    reg [15:0] rx_cnt;
    reg        rx_overflow;
    reg [7:0]  req_buf [0:`DBG_MAX_PAYLOAD-1];
    reg        do_exec;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state    <= RX_MAGIC0;
            rx_cmd      <= 8'h00;
            rx_len      <= 16'h0000;
            rx_cnt      <= 16'h0000;
            rx_overflow <= 1'b0;
            crc_init    <= 1'b0;
            crc_en      <= 1'b0;
            do_exec     <= 1'b0;
        end else begin
            crc_init <= 1'b0;
            crc_en   <= 1'b0;
            do_exec  <= 1'b0;

            case (rx_state)
                RX_MAGIC0: begin
                    if (rx_valid && rx_data == `DBG_MAGIC0) begin
                        crc_init <= 1'b1;
                        rx_state <= RX_MAGIC1;
                    end
                end

                RX_MAGIC1: begin
                    if (rx_valid) rx_state <= (rx_data == `DBG_MAGIC1) ? RX_CMD : RX_MAGIC0;
                end

                RX_CMD: begin
                    if (rx_valid) begin
                        rx_cmd   <= rx_data;
                        crc_en   <= 1'b1;
                        rx_state <= RX_LEN_L;
                    end
                end

                RX_LEN_L: begin
                    if (rx_valid) begin
                        rx_len[7:0] <= rx_data;
                        crc_en      <= 1'b1;
                        rx_state    <= RX_LEN_H;
                    end
                end

                RX_LEN_H: begin
                    if (rx_valid) begin
                        rx_len[15:8] <= rx_data;
                        crc_en       <= 1'b1;
                        rx_cnt       <= 16'h0000;
                        rx_overflow  <= 1'b0;

                        if ({rx_data, rx_len[7:0]} == 16'h0000) rx_state <= RX_CRC;
                        else                                     rx_state <= RX_PAY;
                    end
                end

                RX_PAY: begin
                    if (rx_valid) begin
                        crc_en <= 1'b1;
                        if (rx_cnt < `DBG_MAX_PAYLOAD) req_buf[rx_cnt[3:0]] <= rx_data;
                        else                           rx_overflow <= 1'b1;
                        if (rx_cnt + 1'b1 == rx_len) rx_state <= RX_CRC;
                        else                         rx_cnt   <= rx_cnt + 1'b1;
                    end
                end

                RX_CRC: begin
                    if (rx_valid) begin
                        rx_state <= (rx_data == crc_val && !rx_overflow) ? RX_EXEC : RX_MAGIC0;
                    end
                end

                RX_EXEC: begin
                    do_exec  <= 1'b1;
                    rx_state <= RX_MAGIC0;
                end

                default: rx_state <= RX_MAGIC0;
            endcase
        end
    end

    localparam [1:0] SRC_BUF     = 2'd0,
                     SRC_PARAMS  = 2'd1,
                     SRC_CAPTURE = 2'd2;

    reg [7:0]  resp_cmd;
    reg [15:0] resp_len;
    reg [1:0]  resp_src;
    reg [7:0]  resp_buf [0:`DBG_MAX_PAYLOAD-1];
    reg        resp_req;
    integer live_byte;

    wire [7:0] cap_status = {6'b0, cap_done, cap_armed};

    wire [15:0] pre16 = {req_buf[5], req_buf[4]};

    wire [15:0] trig_idx16 = {{(16-AW){1'b0}}, cap_trig_idx};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_wr_en        <= 1'b0;
            p_wr_addr      <= {PW{1'b0}};
            p_wr_data      <= 32'h0000_0000;
            cap_arm        <= 1'b0;
            cap_disarm     <= 1'b0;
            cap_trig_ch    <= 2'd0;
            cap_trig_level <= {DW{1'b0}};
            cap_trig_edge  <= 2'd0;
            cap_pre_len    <= {AW{1'b0}};
            resp_cmd       <= 8'h00;
            resp_len       <= 16'h0000;
            resp_src       <= SRC_BUF;
            resp_req       <= 1'b0;
        end else begin
            p_wr_en    <= 1'b0;
            cap_arm    <= 1'b0;
            cap_disarm <= 1'b0;
            resp_req   <= 1'b0;

            if (do_exec) begin
                resp_cmd <= rx_cmd | `DBG_RESP_FLAG;
                resp_src <= SRC_BUF;

                case (rx_cmd)
                    `CMD_READ_FUZZY: begin

                        resp_len <= 16'd16;
                        for (live_byte=0; live_byte<16; live_byte=live_byte+1)
                            resp_buf[live_byte] <= fuzzy_data[live_byte*8 +: 8];
                        resp_req <= 1'b1;
                    end
                    `CMD_READ_LIVE: begin

                        resp_len <= CB + 8;
                        for (live_byte=0; live_byte<CB+8; live_byte=live_byte+1)
                            resp_buf[live_byte] <= live_data[live_byte*8 +: 8];
                        resp_req <= 1'b1;
                    end

                    `CMD_PING: begin
                        resp_len    <= 16'd2;
                        resp_buf[0] <= VER[15:8];
                        resp_buf[1] <= VER[7:0];
                        resp_req    <= 1'b1;
                    end

                    `CMD_SET_PARAM: begin
                        p_wr_en   <= 1'b1;
                        p_wr_addr <= req_buf[0][PW-1:0];
                        p_wr_data <= {req_buf[4], req_buf[3], req_buf[2], req_buf[1]};
                        resp_len    <= 16'd1;
                        resp_buf[0] <= req_buf[0];
                        resp_req    <= 1'b1;
                    end

                    `CMD_GET_PARAM: begin

                        resp_len    <= 16'd5;
                        resp_buf[0] <= req_buf[0];
                        resp_buf[1] <= p_rd_data[7:0];
                        resp_buf[2] <= p_rd_data[15:8];
                        resp_buf[3] <= p_rd_data[23:16];
                        resp_buf[4] <= p_rd_data[31:24];
                        resp_req    <= 1'b1;
                    end

                    `CMD_DUMP_PARAMS: begin
                        resp_len <= N_PARAM * 4;
                        resp_src <= SRC_PARAMS;
                        resp_req <= 1'b1;
                    end

                    `CMD_ARM_CAPTURE: begin
                        cap_trig_ch    <= req_buf[0][1:0];
                        cap_trig_edge  <= req_buf[1][1:0];
                        cap_trig_level <= {req_buf[3], req_buf[2]};
                        cap_pre_len    <= pre16[AW-1:0];
                        cap_arm        <= 1'b1;
                        resp_len       <= 16'd1;
                        resp_buf[0]    <= cap_status;
                        resp_req       <= 1'b1;
                    end

                    `CMD_READ_CAPTURE: begin
                        resp_len <= DEPTH * NCH * (DW/8);
                        resp_src <= SRC_CAPTURE;
                        resp_req <= 1'b1;
                    end

                    `CMD_CAP_STATUS: begin
                        resp_len    <= 16'd3;
                        resp_buf[0] <= cap_status;
                        resp_buf[1] <= trig_idx16[7:0];
                        resp_buf[2] <= trig_idx16[15:8];
                        resp_req    <= 1'b1;
                    end

                    `CMD_DISARM: begin
                        cap_disarm  <= 1'b1;
                        resp_len    <= 16'd1;
                        resp_buf[0] <= cap_status;
                        resp_req    <= 1'b1;
                    end

                    default: begin

                        resp_len <= 16'd0;
                        resp_req <= 1'b1;
                    end
                endcase
            end
        end
    end

    localparam [2:0] TX_IDLE = 3'd0,
                     TX_MAG0 = 3'd1,
                     TX_MAG1 = 3'd2,
                     TX_CMD  = 3'd3,
                     TX_LEN  = 3'd4,
                     TX_PAY  = 3'd5,
                     TX_CRC  = 3'd6;

    reg [15:0]       tx_cnt;
    reg [2:0]        tx_state;

    wire gp_read = do_exec && (rx_cmd == `CMD_GET_PARAM);
    assign p_rd_addr = gp_read ? req_buf[0][PW-1:0] :
                       ((tx_state == TX_PAY) && (resp_src == SRC_PARAMS))
                           ? tx_cnt[15:2] : {PW{1'b0}};

    reg [15:0] cap_word_i;
    assign cap_rd_addr = cap_word_i[AW-1:0];

    reg [NCH*DW-1:0] cap_word;

    always @(posedge clk) begin
        cap_word <= cap_rd_data;
    end

    wire [7:0] cap_word_bytes [0:CB-1];
    genvar gi;
    generate
        for (gi = 0; gi < CB; gi = gi + 1) begin : gen_capbyte
            assign cap_word_bytes[gi] = cap_word[gi*8 +: 8];
        end
    endgenerate

    reg        tx_step;
    reg [7:0]  tx_crc;
    reg [3:0]  cap_byte;
    reg        cap_ready;
    reg [1:0]  cap_wait;

    localparam [3:0] CB_LAST = CB - 1;

    wire tx_ready = !tx_busy && !tx_send;

    wire [4:0] pay_sel = {tx_cnt[1:0], 3'b000};

    reg [7:0] pay_byte;
    always @(*) begin
        case (resp_src)
            SRC_PARAMS:  pay_byte = p_rd_data[pay_sel +: 8];
            SRC_CAPTURE: pay_byte = cap_word_bytes[cap_byte];
            default:     pay_byte = (tx_cnt < `DBG_MAX_PAYLOAD) ? resp_buf[tx_cnt[3:0]] : 8'h00;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state  <= TX_IDLE;
            tx_data   <= 8'h00;
            tx_send   <= 1'b0;
            tx_step   <= 1'b0;
            tx_cnt    <= 16'd0;
            tx_crc    <= 8'h00;
            cap_byte  <= 4'd0;
            cap_ready <= 1'b1;
            cap_wait  <= 2'd0;
        end else begin
            tx_send <= 1'b0;

            case (tx_state)

                TX_IDLE: begin
                    if (resp_req && tx_ready) begin
                        tx_crc   <= 8'h00;
                        tx_cnt   <= 16'd0;
                        tx_step  <= 1'b0;
                        tx_state <= TX_MAG0;
                    end
                end

                TX_MAG0: if (tx_ready) begin
                    tx_data  <= `DBG_MAGIC0;
                    tx_send  <= 1'b1;
                    tx_state <= TX_MAG1;
                end

                TX_MAG1: if (tx_ready) begin
                    tx_data  <= `DBG_MAGIC1;
                    tx_send  <= 1'b1;
                    tx_state <= TX_CMD;
                end

                TX_CMD: if (tx_ready) begin
                    tx_data  <= resp_cmd;
                    tx_send  <= 1'b1;
                    tx_crc   <= crc8_step(tx_crc, resp_cmd);
                    tx_state <= TX_LEN;
                end

                TX_LEN: if (tx_ready) begin
                    tx_send <= 1'b1;
                    if (!tx_step) begin
                        tx_data <= resp_len[7:0];
                        tx_crc  <= crc8_step(tx_crc, resp_len[7:0]);
                        tx_step <= 1'b1;
                    end else begin
                        tx_data  <= resp_len[15:8];
                        tx_crc   <= crc8_step(tx_crc, resp_len[15:8]);
                        if (resp_len == 16'd0) begin
                            tx_state <= TX_CRC;
                        end else begin

                            cap_byte   <= 4'd0;
                            cap_word_i <= 16'd0;
                            cap_wait   <= 2'd2;
                            cap_ready  <= (resp_src != SRC_CAPTURE);
                            tx_state   <= TX_PAY;
                        end
                    end
                end

                TX_PAY: begin
                    if (!cap_ready) begin

                        if (cap_wait == 2'd0) cap_ready <= 1'b1;
                        else                  cap_wait  <= cap_wait - 1'b1;
                    end else if (tx_ready) begin
                        tx_data <= pay_byte;
                        tx_send <= 1'b1;
                        tx_crc  <= crc8_step(tx_crc, pay_byte);

                        if (tx_cnt + 1'b1 == resp_len) begin
                            tx_state <= TX_CRC;
                        end else begin
                            tx_cnt <= tx_cnt + 1'b1;
                            if (resp_src == SRC_CAPTURE) begin
                                if (cap_byte == CB_LAST) begin
                                    cap_byte   <= 4'd0;
                                    cap_word_i <= cap_word_i + 1'b1;
                                    cap_wait   <= 2'd2;
                                    cap_ready  <= 1'b0;
                                end else begin
                                    cap_byte <= cap_byte + 1'b1;
                                end
                            end
                        end
                    end
                end

                TX_CRC: if (tx_ready) begin
                    tx_data  <= tx_crc;
                    tx_send  <= 1'b1;
                    tx_state <= TX_IDLE;
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
