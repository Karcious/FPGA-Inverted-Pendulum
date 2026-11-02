`timescale 1ns / 1ps

`include "dbg_pkg.vh"

module tb_curve_protocol;

    localparam integer CLK_FREQ = 1_000_000;
    localparam integer BAUD     = 100_000;
    localparam integer BIT_T    = CLK_FREQ / BAUD;
    localparam integer DEPTH    = 64;
    localparam integer AW       = 6;
    localparam integer NCH      = 4;
    localparam integer DW       = 16;

    localparam integer N_PARAM  = 52;

    localparam integer PRE_LEN  = 4;
    localparam integer TRIG_CH  = 1;
    localparam integer TRIG_LVL = 500;
    localparam integer TRIG_IDX = TRIG_LVL / 100;
    localparam integer N_SAMPLE = DEPTH + 1;

    reg               clk    = 1'b0;
    reg               rst_n  = 1'b0;
    reg               rx_pin = 1'b1;
    wire              tx_pin;
    wire [N_PARAM*32-1:0] param_flat;
    reg               sample_en = 1'b0;
    reg  [NCH*DW-1:0] probe_data = {(NCH*DW){1'b0}};
    wire              cap_armed, cap_done;

    always #500 clk = ~clk;

    dbg_top #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD),
        .NCH      (NCH),
        .DW       (DW),
        .DEPTH    (DEPTH),
        .AW       (AW),
        .N_PARAM  (N_PARAM), .PW(8)

    ) dut (
        .trajectory_readouts({64'd0,32'h00010000,160'd0,32'h12345678,224'd0}),
        .control_status(32'd0),
        .clk          (clk),
        .rst_n        (rst_n),
        .uart_rx_pin  (rx_pin),
        .uart_tx_pin  (tx_pin),
        .param_flat   (param_flat),
        .sample_en    (sample_en),
        .probe_data   (probe_data),
        .cap_armed    (cap_armed),
        .cap_done     (cap_done)
    );

    integer errors = 0;
    integer checks = 0;

    task check;
        input        cond;
        input [255:0] name;
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("  [FAIL] %0s", name);
            end
        end
    endtask

    function [7:0] crcstep;
        input [7:0] c;
        input [7:0] d;
        integer k;
        reg [7:0] x;
        begin
            x = c ^ d;
            for (k = 0; k < 8; k = k + 1)
                x = x[7] ? ((x << 1) ^ 8'h07) : (x << 1);
            crcstep = x;
        end
    endfunction

    task uart_put;
        input [7:0] b;
        integer i;
        begin
            rx_pin = 1'b0;
            repeat (BIT_T) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                rx_pin = b[i];
                repeat (BIT_T) @(posedge clk);
            end
            rx_pin = 1'b1;
            repeat (BIT_T) @(posedge clk);
        end
    endtask

    integer txlog;

    task uart_get;
        output [7:0] b;
        integer i;
        begin
            @(negedge tx_pin);
            repeat (BIT_T + BIT_T/2) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = tx_pin;
                if (i != 7) repeat (BIT_T) @(posedge clk);
            end
            repeat (BIT_T) @(posedge clk);
            $fwrite(txlog, "%02x\n", b);
        end
    endtask

    reg [7:0] txpay [0:1023];
    reg [7:0] rxpay [0:1023];

    task send_frame;
        input [7:0]  cmd;
        input [15:0] len;
        integer k;
        reg [7:0] c;
        begin
            c = 8'h00;
            uart_put(8'hAA);
            uart_put(8'h55);
            uart_put(cmd);       c = crcstep(c, cmd);
            c = crcstep(c, len[7:0]);   uart_put(len[7:0]);
            c = crcstep(c, len[15:8]);  uart_put(len[15:8]);
            for (k = 0; k < len; k = k + 1) begin
                uart_put(txpay[k]);
                c = crcstep(c, txpay[k]);
            end
            uart_put(c);
        end
    endtask

    task recv_frame;
        output [7:0]  cmd;
        output [15:0] len;
        integer k;
        reg [7:0] b, c, c0, c1;
        begin
            uart_get(b); check(b === 8'hAA, "resp magic0");
            uart_get(b); check(b === 8'h55, "resp magic1");
            uart_get(b);
            cmd = b;
            c = crcstep(8'h00, b);
            uart_get(c0); c = crcstep(c, c0);
            uart_get(c1); c = crcstep(c, c1);
            len = {c1, c0};
            for (k = 0; k < len; k = k + 1) begin
                uart_get(b);
                rxpay[k] = b;
                c = crcstep(c, b);
            end
            uart_get(b);
            check(b === c, "resp crc8");
        end
    endtask

    reg [7:0]  rcmd;
    reg [15:0] rlen;
    reg [31:0] v;

    task get_param;
        input [7:0] addr;
        input [31:0] want;
        begin
            txpay[0] = addr;
            send_frame(`CMD_GET_PARAM, 1);
            recv_frame(rcmd, rlen);
            check(rcmd === (`CMD_GET_PARAM | `DBG_RESP_FLAG), "GET_PARAM resp cmd");
            check(rlen === 16'd5, "GET_PARAM resp len");
            check(rxpay[0] === addr, "GET_PARAM echo addr");
            v = {rxpay[4], rxpay[3], rxpay[2], rxpay[1]};
            check(v === want, "GET_PARAM value");
            if (v !== want) $display("      want %08x got %08x", want, v);
        end
    endtask

    task set_param;
        input [7:0]  addr;
        input [31:0] val;
        begin
            txpay[0] = addr;
            txpay[1] = val[7:0];
            txpay[2] = val[15:8];
            txpay[3] = val[23:16];
            txpay[4] = val[31:24];
            send_frame(`CMD_SET_PARAM, 5);
            recv_frame(rcmd, rlen);
            check(rcmd === (`CMD_SET_PARAM | `DBG_RESP_FLAG), "SET_PARAM resp cmd");
            check(rxpay[0] === addr, "SET_PARAM echo addr");
        end
    endtask

    task push_sample;
        input integer i;
        begin
            probe_data[(0*DW) +: DW] = i[15:0];
            probe_data[(1*DW) +: DW] = i * 100;
            probe_data[(2*DW) +: DW] = 16'h1000 + i[15:0];
            probe_data[(3*DW) +: DW] = 16'h2000 + i[15:0];
            @(posedge clk);
            sample_en = 1'b1;
            @(posedge clk);
            sample_en = 1'b0;
        end
    endtask

    integer i, base;
    reg [15:0] w;

    initial begin
        if ($test$plusargs("vcd")) begin

            $dumpfile("tb_curve_protocol.vcd");
            $dumpvars(0, tb_curve_protocol);
        end

        txlog = $fopen("dut_tx.hex", "w");
        if (txlog == 0) begin
            $display("*** 打不开 dut_tx.hex，上位机回放测试会失败 ***");
            errors = errors + 1;
        end

        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);

        $display("\n== 1. PING ==");
        send_frame(`CMD_PING, 0);
        recv_frame(rcmd, rlen);
        check(rcmd === (`CMD_PING | `DBG_RESP_FLAG), "PING resp cmd");
        check(rlen === 16'd2, "PING resp len");

        check(rxpay[0] === dut.VER[15:8] && rxpay[1] === dut.VER[7:0], "PING version");

        get_param(8'd71,32'h12345678);
        get_param(8'd77,32'h00010000);
        get_param(8'd79,32'd0);
        set_param(8'd64,32'd2);
        get_param(8'd0,{8'd0,dut.VER,dut.REV});
        set_param(8'd65,32'd3000);
        get_param(8'd1,32'd1000);
        $display("== 2. GET_PARAM 默认值 ==");
        get_param(`P_CTRL_HZ,  32'd1000);
        get_param(`P_KP_BAL,   32'h0004_8000);

        get_param(`P_KD_BAL,   32'h0078_0000);

        get_param(`P_KI_ARM,   32'hFFFF_FC29);

        get_param(`P_FILT_SEL, 32'd271);
        get_param(`P_MAGIC,    {8'd0, dut.VER, dut.REV});

        get_param(`P_KD_ARM,   32'hFFD8_0000);

        get_param(`P_WAITUP_BRAKE, 32'd0);

        get_param(`P_GAIN_A_KP,   32'hFFFE_CCCD);
        get_param(`P_GAIN_A_KD,   32'hFFD8_0000);
        get_param(`P_GAIN_B_KP,   32'hFFFE_CCCD);
        get_param(`P_GAIN_B_KD,   32'hFFD8_0000);
        get_param(`P_PHASE_A_MS,  32'd0);
        get_param(`P_PHASE_B_MS,  32'd0);

        get_param(`P_SETTLE_DAMP, 32'h0002_0000);

        $display("== 3. SET_PARAM / 回读 ==");
        set_param(`P_KP_BAL, 32'h0002_5800);
        get_param(`P_KP_BAL, 32'h0002_5800);

        set_param(`P_POS_SLEW, 32'hFFFF_FF00);
        get_param(`P_POS_SLEW, 32'hFFFF_FF00);

        $display("== 4. DUMP_PARAMS ==");
        send_frame(`CMD_DUMP_PARAMS, 0);
        recv_frame(rcmd, rlen);
        check(rcmd === (`CMD_DUMP_PARAMS | `DBG_RESP_FLAG), "DUMP resp cmd");
        check(rlen === N_PARAM*4, "DUMP resp len");

        base = `P_KP_BAL * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'h0002_5800, "DUMP KP_BAL");

        check({rxpay[3], rxpay[2], rxpay[1], rxpay[0]} === {8'd0, dut.VER, dut.REV}, "DUMP MAGIC");

        base = `P_WAITUP_BRAKE * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'd0, "DUMP WAITUP_BRAKE default 0");
        base = `P_PHASE_B_MS * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'd0, "DUMP PHASE_B_MS default 0");

        base = `P_GAIN_A_KP * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'hFFFE_CCCD, "DUMP GAIN_A_KP default -1.2");

        base = `P_SETTLE_DAMP * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'h0002_0000, "DUMP SETTLE_DAMP default +2.0");

        base = `P_MOVE_SLEW * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'd13107, "DUMP MOVE_SLEW user default 0.2");
        base = `P_PROF_SLEW * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'd3787, "DUMP PROF_SLEW default 3787 (= 老 MAX_SLEW_Q16)");
        base = `P_MOVE_ACC * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'd98, "DUMP MOVE_ACC user default 0.0015");
        base = `P_SPIN_W * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'd0, "DUMP SPIN_W default 0 (不转)");
        base = `P_SPIN_KP * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'hFFF5_0000, "DUMP SPIN_KP default -11.0");
        base = `P_SPIN_KI * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'd0, "DUMP SPIN_KI default 0 (纯 P)");

        base = `P_SPIN_ACC * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'd4, "DUMP SPIN_ACC default 4 (表尾)");

        base = `P_HOME_RAMP * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'h0000_4000, "DUMP HOME_RAMP default 0x4000 (0x0207 起 = 参考 250 码/s)");
        base = `P_ARM_MAX * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'd0, "DUMP ARM_MAX default 0 (= 关，冷启动逐位等于 0x0200)");

        base = `P_MOVE_DEC * 4;
        check({rxpay[base+3], rxpay[base+2], rxpay[base+1], rxpay[base]}
              === 32'd30, "DUMP MOVE_DEC default deceleration 30");

        $display("== 5. ARM_CAPTURE + 触发 + 读回 ==");
        txpay[0] = TRIG_CH;
        txpay[1] = 2'd0;
        txpay[2] = TRIG_LVL[7:0];
        txpay[3] = TRIG_LVL[15:8];
        txpay[4] = PRE_LEN[7:0];
        txpay[5] = PRE_LEN[15:8];
        send_frame(`CMD_ARM_CAPTURE, 6);
        recv_frame(rcmd, rlen);
        check(rcmd === (`CMD_ARM_CAPTURE | `DBG_RESP_FLAG), "ARM resp cmd");
        check(rlen === 16'd1, "ARM resp len");
        check(rxpay[0][0] === 1'b0, "arm 前 armed 应为 0");

        repeat (5) @(posedge clk);
        check(cap_armed === 1'b1, "arm 后 armed 应为 1");

        for (i = 0; i < N_SAMPLE; i = i + 1) push_sample(i);

        for (i = 0; i < 200 && !cap_done; i = i + 1) @(posedge clk);
        check(cap_done === 1'b1, "采满后 done 应为 1");
        check(cap_armed === 1'b0, "done 后 armed 应清零");

        send_frame(`CMD_CAP_STATUS, 0);
        recv_frame(rcmd, rlen);
        check(rlen === 16'd3, "CAP_STATUS resp len");
        check(rxpay[0][1] === 1'b1, "CAP_STATUS done 位");
        w = {rxpay[2][1:0], rxpay[1]};
        check(w === TRIG_IDX[AW-1:0], "trig_idx 应为触发采样索引");
        if (w !== TRIG_IDX[AW-1:0])
            $display("      want %0d got %0d", TRIG_IDX, w);

        send_frame(`CMD_READ_CAPTURE, 0);
        recv_frame(rcmd, rlen);
        check(rcmd === (`CMD_READ_CAPTURE | `DBG_RESP_FLAG), "READ_CAPTURE resp cmd");
        check(rlen === DEPTH*NCH*(DW/8), "READ_CAPTURE resp len");

        base = PRE_LEN * (NCH*(DW/8));
        w = {rxpay[base + TRIG_CH*2 + 1], rxpay[base + TRIG_CH*2]};
        check(w === TRIG_LVL[15:0], "触发点通道值应等于触发电平");
        if (w !== TRIG_LVL[15:0])
            $display("      want %0d got %0d", TRIG_LVL, w);

        w = {rxpay[1], rxpay[0]};
        check(w === (TRIG_IDX - PRE_LEN), "逻辑 0 号点的 ch0 序号");
        if (w !== (TRIG_IDX - PRE_LEN))
            $display("      want %0d got %0d", TRIG_IDX - PRE_LEN, w);

        for (i = 1; i < DEPTH; i = i + 1) begin
            base = i * (NCH*(DW/8));
            w = {rxpay[base+1], rxpay[base]};
            if (w !== ((TRIG_IDX - PRE_LEN + i) & 16'hFFFF)) begin
                check(1'b0, "采集数据不连贯");
                $display("      逻辑 %0d 号点 want %0d got %0d",
                          i, (TRIG_IDX - PRE_LEN + i) & 16'hFFFF, w);
                i = DEPTH;
            end
        end
        check(1'b1, "采集数据连贯性");

        $display("== 6. DISARM ==");
        send_frame(`CMD_ARM_CAPTURE, 6);
        recv_frame(rcmd, rlen);
        repeat (5) @(posedge clk);
        check(cap_armed === 1'b1, "re-arm 后 armed");
        send_frame(`CMD_DISARM, 0);
        recv_frame(rcmd, rlen);
        repeat (5) @(posedge clk);
        check(cap_armed === 1'b0, "DISARM 后 armed 清零");

        $display("== 7. 坏 CRC 应被丢弃 ==");

        uart_put(8'hAA); uart_put(8'h55); uart_put(`CMD_PING);
        uart_put(8'h00); uart_put(8'h00); uart_put(8'hDE);
        repeat (4000) @(posedge clk);
        check(tx_pin === 1'b1, "坏 CRC 的帧不应产生响应");

        $display("\n================================");

        $display("  RTL checks = %0d, errors = %0d", checks, errors);
        if (errors == 0) $display("  *** PASS ***");
        else             $display("  *** FAIL ***");
        $display("================================\n");
        $fclose(txlog);
        $finish;
    end

    initial begin
        #500_000_000;
        $display("*** TIMEOUT ***");
        $finish;
    end

endmodule
