`timescale 1ns/1ps

module tb_catch_frame;

    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;

    reg signed [31:0] encoder = 10000;
    reg active  = 0;
    reg session = 0;
    reg balance_confirmed = 0;
    reg fault_in = 0;

    reg signed [31:0] hslew = 32'sd0;
    integer lag;

    reg [7:0] tick_cnt = 0;
    wire ctrl_tick = (tick_cnt == 8'd31);
    always @(posedge clk) tick_cnt <= ctrl_tick ? 8'd0 : tick_cnt + 8'd1;

    wire signed [15:0] pos_a;
    wire signed [31:0] err_a, tgt_a, tcmd_a;
    wire ol_a, flt_a, rdy_a, hf_a, hl_a;
    wire rebase_a = ol_a;

    position_feedback #(.LATCH_ON_ENTRY(1)) fb_a (
        .clk(clk), .rst_n(rst_n), .active(active), .session(session),
        .re_zero(1'b0), .encoder(encoder), .target(tgt_a),
        .relative_pos(pos_a), .error_q16(err_a), .fault(flt_a),

        .leg_active(1'b0), .leg_pos(),
        .origin_latched(ol_a));

    key_motion_target km_a (
        .clk(clk), .rst_n(rst_n), .tick(ctrl_tick), .active(active),
        .balance_confirmed(balance_confirmed), .re_zero(rebase_a), .fault(flt_a),
        .key_neg(1'b0), .key_zero(1'b0), .key_pos(1'b0),
        .remote_move_req(1'b0), .remote_move_counts(16'sd0),
        .profile_request(1'b0), .profile_target(32'sd0), .profile_slew(32'sd0),
        .hold_motion(1'b0),
        .angle_error(16'sd0), .actual_pos(pos_a), .dpos(16'sd0),
        .slew_sel(32'sd0), .prof_slew_sel(32'sd0), .accel_sel(32'sd0),.decel_sel(32'sd0),
        .home_slew_sel(hslew),
        .target_ref(tgt_a), .target_cmd(tcmd_a), .ready(rdy_a),
        .moving(), .paused(), .reached(), .homing(), .home_fault(hf_a),
        .at_target(), .traveling(), .recovering(), .home_leg(hl_a));

    wire signed [15:0] pos_b;
    wire signed [31:0] err_b, tgt_b, tcmd_b;
    wire ol_b, flt_b, rdy_b, hf_b, hl_b;
    wire rebase_b = 1'b0;

    position_feedback #(.LATCH_ON_ENTRY(1)) fb_b (
        .clk(clk), .rst_n(rst_n), .active(active), .session(session),
        .re_zero(1'b0), .encoder(encoder), .target(tgt_b),
        .relative_pos(pos_b), .error_q16(err_b), .fault(flt_b),
        .leg_active(1'b0), .leg_pos(),
        .origin_latched(ol_b));

    key_motion_target km_b (
        .clk(clk), .rst_n(rst_n), .tick(ctrl_tick), .active(active),
        .balance_confirmed(balance_confirmed), .re_zero(rebase_b), .fault(flt_b),
        .key_neg(1'b0), .key_zero(1'b0), .key_pos(1'b0),
        .remote_move_req(1'b0), .remote_move_counts(16'sd0),
        .profile_request(1'b0), .profile_target(32'sd0), .profile_slew(32'sd0),
        .hold_motion(1'b0),
        .angle_error(16'sd0), .actual_pos(pos_b), .dpos(16'sd0),
        .slew_sel(32'sd0), .prof_slew_sel(32'sd0), .accel_sel(32'sd0),.decel_sel(32'sd0),
        .home_slew_sel(hslew),
        .target_ref(tgt_b), .target_cmd(tcmd_b), .ready(rdy_b),
        .moving(), .paused(), .reached(), .homing(), .home_fault(hf_b),
        .at_target(), .traveling(), .recovering(), .home_leg(hl_b));

    integer checks = 0, k;
    task check; input ok; input [1023:0] what;
        begin
            checks = checks + 1;
            if (ok !== 1'b1)
                $fatal(1, "catch_frame #%0d FAILED: %0s  (A: pos=%0d tgt=%0d err=%0d ol=%b | B: pos=%0d tgt=%0d err=%0d ol=%b)",
                       checks, what, pos_a, tgt_a >>> 16, err_a >>> 16, ol_a,
                       pos_b, tgt_b >>> 16, err_b >>> 16, ol_b);
        end
    endtask

    initial begin
        $display("");
        $display("  起摆：摆臂从 0 码扫到 +395 码（相对上电原点 ORIGIN_INIT=10000）");
        repeat(4) @(negedge clk);
        encoder = 10000; session = 1; active = 0;
        repeat(4) @(negedge clk);
        rst_n = 1;
        repeat(4) @(negedge clk);

        for (k = 0; k < 395; k = k + 1) begin
            encoder = encoder + 1;
            @(negedge clk);
        end
        $display("   摆臂就位 encoder=%0d，两条路位移都还是 +395 码", encoder);
        check(pos_a === 16'sd395 && pos_b === 16'sd395, "接管前位移应为 +395");
        check(err_a === 32'sd0  && err_b === 32'sd0,  "接管前误差应为 0");

        @(negedge clk);
        active = 1; balance_confirmed = 1;
        #1;
        $display("");
        $display("  ── 接住那一拍 ──");
        $display("     origin_latched:  A=%b   B=%b   （两条路都在喊）", ol_a, ol_b);
        check(ol_a === 1'b1, "★① 应在本拍开火（A 路）");
        check(ol_b === 1'b1, "★① 应在本拍开火（B 路）");

        repeat(4) @(negedge clk);
        $display("     锁完原点：       A: pos=%0d  B: pos=%0d", pos_a, pos_b);
        check(pos_a === 16'sd0 && pos_b === 16'sd0, "★① 拍后两边位移都应归 0");

        $display("     target_ref:      A=%0d 码   B=%0d 码", tgt_a >>> 16, tgt_b >>> 16);
        check(tgt_a === 32'sd0,            "修复路 target 应被同拍清零");
        check(tgt_b === (32'sd395 <<< 16), "对照路 target 应冻在旧坐标系的 +395");

        $display("     error_q16:       A=%0d 码   B=%0d 码", err_a >>> 16, err_b >>> 16);
        $display("     ⟹ 外环接管第一拍：A 修好了 = 0 码；B 是 137° 的假误差");
        check(err_a === 32'sd0, "★ 修复路误差必须为 0");
        check(err_b !== 32'sd0, "★ 变异敏感度：未修复路必须能看出假误差");

        @(negedge clk);
        active = 0; balance_confirmed = 0;
        repeat(60) @(negedge clk);
        encoder = encoder - 300;
        repeat(60) @(negedge clk);
        @(negedge clk);
        active = 1; balance_confirmed = 1;
        #1;
        $display("");
        $display("  ── 同一会话第二次接住 ──");
        $display("     origin_latched:  A=%b   B=%b   （homed 锁死，都不该再喊）",
                 ol_a, ol_b);
        check(ol_a === 1'b0 && ol_b === 1'b0, "本次会话的第二次进平衡态不该再锁原点");
        repeat(4) @(negedge clk);
        $display("     error_q16:       A=%0d 码   B=%0d 码", err_a >>> 16, err_b >>> 16);
        check(err_a === 32'sd0 && err_b === 32'sd0,
              "第二次接住两边误差都该是 0（和现场数据吻合）");

        hslew = 32'sd0;
        @(negedge clk);
        active = 0; balance_confirmed = 0;
        repeat(64) @(negedge clk);
        @(negedge clk);
        active = 1; balance_confirmed = 1;

        repeat(100*32) @(negedge clk);
        $display("");
        $display("  ── ★③ 资格考试：关着，只等 100 ms ──");
        $display("     pos=%0d  target_ref=%0d  home_leg=%b", pos_a, tgt_a>>>16, hl_a);
        check(hl_a === 1'b0, "关着时 100 ms 还不该够资格（原版 READY_TICKS=300）");

        check(($signed(tgt_a) >>> 16) === -16'sd300,
              "关着时 100 ms 参考必须还冻在 -300（动过 = 资格考试漏到关档了）");
        repeat(205*32) @(negedge clk);
        $display("  ── ★③ 回家：关（hslew=0 = `set 47 0`） ──");
        $display("     pos=%0d  target_ref=%0d  home_leg=%b", pos_a, tgt_a>>>16, hl_a);
        check(hl_a === 1'b0,                        "关着时 home_leg 必须恒 0（硬规矩 #1）");
        check(tgt_a !== 32'sd0,                     "关着时参考不该跳，应该还在爬");

        hslew = 32'h0000_4000;
        @(negedge clk);
        active = 0; balance_confirmed = 0;
        repeat(64) @(negedge clk);
        @(negedge clk);
        active = 1; balance_confirmed = 1;

        repeat(100*32) @(negedge clk);
        $display("  ── ★③ 资格考试：开着，只等 100 ms ──");
        $display("     pos=%0d  target_ref=%0d  home_leg=%b", pos_a, tgt_a>>>16, hl_a);
        check(hl_a === 1'b1,   "开着时 100 ms 就该够资格（放松版 40 ms）");
        check(pos_a !== 16'sd0,"此时摆臂还离得远（这一节才有意义）");

        $display("     ⟹ 参考该已经从 -300 往 0 爬了一段（爬了多少 = 速度）");
        check($signed(tgt_a) > (-16'sd300 <<< 16),
              "开着时参考必须已经离开 -300（爬起来了）");
        check($signed(tgt_a) < 32'sd0,
              "开着时参考**不该**当场落到 0（那就是 0x0206 那个错的跳变）");
        repeat(205*32) @(negedge clk);
        $display("  ── ★③ 回家：开（hslew=0x4000） ──");
        $display("     pos=%0d  target_ref=%0d  home_leg=%b", pos_a, tgt_a>>>16, hl_a);
        check(hl_a === 1'b1,   "开着时 home_leg 应该拉起来");
        check($signed(tgt_a) < 32'sd0, "参考还在路上（摆臂没动，它爬到领先 64 码就等）");

        check($signed(tgt_a) > (-16'sd260 <<< 16) && $signed(tgt_a) < (-16'sd200 <<< 16),
              "绳子放松了：参考应该领跑到 −260…−200（−283 = 还是 16 码的老绳子）");

        for (k = 0; k < 4000; k = k + 1) begin
            repeat(32) @(negedge clk);
            lag = $signed(tgt_a[31:16]) - $signed(pos_a);
            if (lag > 8) begin
                encoder = encoder + (lag - 8);
                @(negedge clk);
            end
            if (hl_a === 1'b0) k = 4000;
        end
        $display("  ── ★③ 摆臂到家 ──");
        $display("     pos=%0d  target_ref=%0d  home_leg=%b", pos_a, tgt_a>>>16, hl_a);
        check(pos_a <= 16'sd16 && pos_a >= -16'sd16, "摆臂应回到 ±16 码以内");
        check(hl_a === 1'b0,    "到家停稳后 home_leg 应该自己放掉");

        $display("");
        $display("  ALL PASS  (%0d checks)  —— 病根：★① 锁原点没通知 key_motion_target", checks);
        $display("");
        $finish;
    end

    initial begin
        #2000000;
        $display("TIMEOUT");
        $fatal(1, "catch_frame TIMEOUT");
    end
endmodule
