`timescale 1ns/1ps

module tb_fold_step;

    reg clk = 0;
    always #10 clk = ~clk;

    reg rst_n = 0;

    reg  signed [31:0] encoder = 10000;
    reg         active  = 0;
    reg         session = 0;
    reg         re_zero = 0;
    wire signed [15:0] pos;
    wire signed [31:0] err_folded;
    wire               fault;

    position_feedback u_fb (
        .clk(clk), .rst_n(rst_n), .active(active), .session(session),
        .re_zero(re_zero), .encoder(encoder),
        .target(32'sd0),
        .relative_pos(pos), .error_q16(err_folded), .fault(fault),

        .leg_active(1'b0), .leg_pos()
    );

    reg [7:0] tick_cnt = 0;
    wire outer_tick = (tick_cnt == 8'd31);
    always @(posedge clk) tick_cnt <= outer_tick ? 8'd0 : tick_cnt + 8'd1;

    wire signed [31:0] a_new = -err_folded;

    wire signed [32:0] disp_raw = {encoder[31], encoder} - 33'sd10000;
    wire signed [32:0] tl_clamp = (disp_raw >  33'sd8191) ?  33'sd8191 :
                                 (disp_raw < -33'sd8191) ? -33'sd8191 : disp_raw;
    reg  signed [31:0] a_old_r = 0;
    reg  signed [31:0] e_old_r = 0;
    always @(posedge clk) if (outer_tick) begin
        a_old_r <=  $signed({tl_clamp[31:0], 16'd0});
        e_old_r <= -$signed({tl_clamp[31:0], 16'd0});
    end

    localparam signed [31:0] KP = 32'hFFFE_8000;
    localparam signed [31:0] KD = 32'hFFE0_0000;

    wire signed [31:0] out_new, out_old;

    pid_q16 u_new (
        .clk(clk), .rst_n(rst_n), .en(outer_tick), .clear(1'b0),
        .target(32'sd0), .actual(a_new),
        .kp(KP), .ki(32'sd0), .kd(KD),
        .out_max(32'sd750 <<< 16), .out_min(-(32'sd750 <<< 16)),
        .en_dlpf(1'b1), .en_dspan2(1'b1), .en_dspan4(1'b0), .lpf_shift(2'd0),
        .out(out_new)
    );

    pid_q16 u_old (
        .clk(clk), .rst_n(rst_n), .en(outer_tick), .clear(1'b0),
        .target(32'sd0), .actual(a_old_r),
        .kp(KP), .ki(32'sd0), .kd(KD),
        .out_max(32'sd750 <<< 16), .out_min(-(32'sd750 <<< 16)),
        .en_dlpf(1'b1), .en_dspan2(1'b1), .en_dspan4(1'b0), .lpf_shift(2'd0),
        .out(out_old)
    );

    integer i, k, n;
    reg signed [31:0] prev_err = 0;

    wire signed [32:0] disp_live = {encoder[31], encoder} - 33'sd10000;
    wire signed [31:0] disp_q16  = {disp_live[15:0], 16'd0};
    integer n_jump = 0;
    reg signed [31:0] delta_e, max_jump = 32'sd0;
    integer checks = 0;
    task check; input ok; input [1023:0] what;
        begin
            checks = checks + 1;
            if (ok !== 1'b1)
                $fatal(1, "fold_step #%0d FAILED: %0s  (enc=%0d disp=%0d err=%0d max_jump=%0d n_jump=%0d)",
                       checks, what, encoder, $signed(disp_live), $signed(err_folded),
                       max_jump >>> 16, n_jump);
        end
    endtask

    task one_tick;
        input integer step;
        begin
            for (k = 0; k < 32; k = k + 1) begin
                if (k < step) encoder = encoder + 1;
                @(negedge clk);
            end
        end
    endtask

    initial begin
        $display("");
        $display(" 拍    enc   pos   err折返   外环新  |  err队友   外环旧");
        $display("------ ------ ----- --------- -------- | --------- --------");
        repeat(4) @(negedge clk);
        encoder = 10000; session = 1; active = 0;
        repeat(4) @(negedge clk);
        rst_n = 1;
        repeat(4) @(negedge clk);
        active = 1;
        repeat(80) @(negedge clk);
        n = 0;

        for (i = 0; i < 178; i = i + 1) begin
            one_tick(4);
            n = n + 1;

            if ($signed(err_folded) !== prev_err || (n % 8) == 0)
                $display(" %4d %6d %5d %9d %8d | %9d %8d",
                         n, encoder, pos, $signed(err_folded), $signed(out_new) >>> 16,
                         $signed(e_old_r), $signed(out_old) >>> 16);

            if (disp_live <= 33'sd520 && disp_live >= -33'sd520)
                check($signed(err_folded) === -disp_q16,
                      "|位移| <= 520 时折返不该改误差");

            check($signed(err_folded) <= (32'sd520 <<< 16) &&
                  $signed(err_folded) >= -(32'sd520 <<< 16),
                  "折返后的误差必须留在半圈以内");

            delta_e = $signed(err_folded) - prev_err;
            if (delta_e < 0) delta_e = -delta_e;
            if (delta_e > max_jump) max_jump = delta_e;
            if (delta_e > (32'sd600 <<< 16)) n_jump = n_jump + 1;
            prev_err = $signed(err_folded);
        end
        $display("--- 走完 %0d 拍，encoder=%0d，位移=%0d 码 ---", n, encoder, encoder-10000);
        $display("");
        $display("  折返那一拍的最大跳幅 = %0d 码（一个整圈 = 1040）", max_jump >>> 16);
        $display("  整个行程里出现 %0d 次「一跳 > 600 码」", n_jump);

        check(max_jump >= (32'sd1030 <<< 16) && max_jump <= (32'sd1040 <<< 16),
              "过半圈那一拍必须是折返先跳一个整圈（不是 ★② 搬家的 972）");
        check(n_jump == 1, "1200 码行程里只该折返一次");
        $display("");
        $display("  ALL PASS  (%0d checks)", checks);
        $display("");
        $finish;
    end

    initial begin
        #200000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
