`timescale 1ns / 1ps

module tb_pid_q16;

    reg clk = 0;
    always #10 clk = ~clk;

    reg rst_n = 0;
    reg en    = 0;
    reg clear = 0;

    reg signed [31:0] target  = 0;
    reg signed [31:0] actual  = 0;
    reg signed [31:0] kp      = 0;
    reg signed [31:0] ki      = 0;
    reg signed [31:0] kd      = 0;
    reg signed [31:0] out_max = 0;
    reg signed [31:0] out_min = 0;

    reg en_dlpf   = 0;
    reg en_dspan2 = 0;
    reg en_dspan4 = 0;
    reg [1:0] lpf_shift = 2'd0;

    wire signed [31:0] out;

    pid_q16 dut (
        .clk(clk), .rst_n(rst_n),
        .en(en), .clear(clear),
        .target(target), .actual(actual),
        .kp(kp), .ki(ki), .kd(kd),
        .out_max(out_max), .out_min(out_min),
        .en_dlpf(en_dlpf), .en_dspan2(en_dspan2),
        .en_dspan4(en_dspan4), .lpf_shift(lpf_shift),
        .out(out)
    );

    wire signed [31:0] out_lpf;

    pid_q16 dut_lpf (
        .clk(clk), .rst_n(rst_n),
        .en(en), .clear(clear),
        .target(target), .actual(actual),
        .kp(kp), .ki(ki), .kd(kd),
        .out_max(out_max), .out_min(out_min),
        .en_dlpf(1'b1), .en_dspan2(1'b0),

        .en_dspan4(1'b0), .lpf_shift(2'd0),
        .out(out_lpf)
    );

    localparam signed [31:0] BIG = 32'sh4000_0000;

    integer checks = 0;
    integer errors = 0;

    task check;
        input              cond;
        input [8*56-1:0]   name;
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("  [FAIL] %0s  (out=%h)", name, out);
            end
        end
    endtask

    task check_lpf;
        input              cond;
        input [8*56-1:0]   name;
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("  [FAIL] %0s  (out_lpf=%h)", name, out_lpf);
            end
        end
    endtask

    task one_tick;
        begin
            @(negedge clk);
            en = 1'b1;
            @(negedge clk);
            en = 1'b0;
            repeat (7) @(negedge clk);
        end
    endtask

    task reset_dut;
        begin
            rst_n   = 1'b0;
            en      = 1'b0;
            clear   = 1'b0;
            target  = 0;  actual = 0;
            kp      = 0;  ki = 0;  kd = 0;
            out_max = BIG;  out_min = -BIG;

            en_dlpf = 0;  en_dspan2 = 0;
            en_dspan4 = 0;  lpf_shift = 2'd0;
            repeat (5) @(negedge clk);
            rst_n   = 1'b1;
            repeat (3) @(negedge clk);
        end
    endtask

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_pid_q16.vcd");
            $dumpvars(0, tb_pid_q16);
        end

        $display("== 1. pure P ==");

        reset_dut();
        kp = 32'sh0001_0000;
        target = 32'sh0002_0000;
        actual = 0;
        one_tick();

        check(out == 32'sh0002_0000, "P: 1.0 * 2.0 -> 2.0");

        $display("== 2. output limiting ==");

        reset_dut();
        kp = 32'sh0001_0000;
        out_max = 32'sh0001_0000;
        out_min = -BIG;
        target = 32'sh0002_0000;
        actual = 0;
        one_tick();
        check(out == 32'sh0001_0000, "limit: 2.0 clamped to out_max 1.0");

        reset_dut();
        kp = 32'sh0001_0000;
        out_max = BIG;
        out_min = -32'sh0001_0000;
        target = -32'sh0002_0000;
        actual = 0;
        one_tick();
        check(out == -32'sh0001_0000, "limit: -2.0 clamped to out_min -1.0");

        reset_dut();
        kp = 32'sh0001_0000;
        out_max = 32'sh0001_0000;
        out_min = -32'sh0003_0000;
        target = 32'sh0002_0000;  actual = 0;
        one_tick();
        check(out == 32'sh0001_0000, "limit: asymmetric high end");
        target = -32'sh0002_0000;
        one_tick();
        check(out == -32'sh0002_0000, "limit: asymmetric low end passes through");

        $display("== 3. pure D: output only on error change ==");

        reset_dut();
        kd = 32'sh0001_0000;
        target = 32'sh0002_0000;
        actual = 0;
        one_tick();

        check(out == 32'sh0002_0000, "D: first tick e=2.0 e_prev=0 -> 2.0");
        one_tick();

        check(out == 0, "D: second tick error unchanged -> 0");
        target = 32'sh0003_0000;
        one_tick();

        check(out == 32'sh0001_0000, "D: step 2.0->3.0 -> 1.0");

        $display("== 4. pure I: accumulate each tick ==");

        reset_dut();
        ki = 32'sh0001_0000;
        target = 32'sh0002_0000;
        actual = 0;
        one_tick();
        check(out == 32'sh0002_0000, "I: tick1 integ=2.0");
        one_tick();
        check(out == 32'sh0004_0000, "I: tick2 integ=4.0");
        one_tick();
        check(out == 32'sh0006_0000, "I: tick3 integ=6.0");

        $display("== 5. integral clamp + conditional integration ==");

        reset_dut();
        kp = -32'sh000A_0000;
        ki = 32'sh0001_0000;
        out_max = 32'sh0004_0000;
        out_min = -32'sh0004_0000;
        target = 32'sh0002_0000;
        actual = 0;

        repeat (5) one_tick();
        check(out == -32'sh0004_0000, "windup: P pins output at out_min");

        kp = 0;
        out_max = BIG;
        out_min = -BIG;
        target = 0;
        actual = 0;
        one_tick();
        check(out == 32'sh0004_0000,
              "windup: integral clamped at out_max (4.0, not 10.0)");

        reset_dut();
        ki = 32'sh0001_0000;
        out_max = 32'sh0004_0000;
        out_min = -32'sh0004_0000;
        target = 32'sh0002_0000;
        actual = 0;
        one_tick();
        one_tick();
        check(out == 32'sh0004_0000, "windup: output reaches out_max");
        repeat (3) one_tick();
        check(out == 32'sh0004_0000, "windup: stays at out_max, no runaway");
        kp = 0;
        out_max = BIG;
        out_min = -BIG;
        target = 0;
        actual = 0;
        one_tick();
        check(out == 32'sh0004_0000,
              "windup: integral still 4.0 after 3 more ticks");

        reset_dut();
        ki = 32'sh0001_0000;
        out_max = 32'sh0004_0000;
        out_min = -32'sh0004_0000;
        target = 32'sh0002_0000;
        actual = 0;
        repeat (4) one_tick();
        target = -32'sh0002_0000;
        one_tick();
        check(out == 32'sh0002_0000, "windup: reverses out cleanly");

        $display("== 6. clear: zero the integral ==");

        reset_dut();
        ki = 32'sh0001_0000;
        target = 32'sh0002_0000;
        actual = 0;
        repeat (3) one_tick();
        check(out == 32'sh0006_0000, "clear: integral built up to 6.0");

        clear = 1'b1;
        repeat (3) @(negedge clk);
        clear = 1'b0;
        one_tick();

        check(out == 32'sh0002_0000, "clear: integral zeroed then restarts at 2.0");

        $display("== 7. clear must not fake a derivative spike ==");

        reset_dut();
        kd = 32'sh0001_0000;
        target = 32'sh0002_0000;
        actual = 0;
        one_tick();
        one_tick();
        check(out == 0, "clear: baseline derivative is 0");

        clear = 1'b1;
        target = 32'sh0064_0000;
        one_tick();
        check(out == 0, "clear: no fake derivative spike on target jump");

        clear = 1'b0;
        one_tick();
        check(out == 0, "clear: derivative still 0 while error holds");

        $display("== 8. Ki=0: no accumulation (degenerates to PD) ==");

        reset_dut();
        kp = 32'sh0001_0000;
        ki = 0;
        target = 32'sh0001_0000;
        actual = 0;
        one_tick();
        check(out == 32'sh0001_0000, "PD: tick1 -> 1.0");
        one_tick();
        check(out == 32'sh0001_0000, "PD: tick2 -> 1.0 (not accumulating)");
        one_tick();
        check(out == 32'sh0001_0000, "PD: tick3 -> 1.0 (not accumulating)");

        $display("== 9. clear does not mute P / D ==");

        reset_dut();
        kp = 32'sh0001_0000;
        target = 32'sh0002_0000;
        actual = 0;
        clear = 1'b1;
        one_tick();
        check(out == 32'sh0002_0000, "clear: P term still active");
        clear = 1'b0;

        $display("== 10. latency: valid 6 clk after en ==");

        reset_dut();
        kp = 32'sh0001_0000;
        target = 32'sh0001_0000;
        actual = 0;
        @(negedge clk);
        en = 1'b1;
        @(negedge clk);
        en = 1'b0;
        check(out == 0, "latency: unchanged 1 clk after en");
        @(negedge clk);
        check(out == 0, "latency: unchanged 2 clk after en");
        @(negedge clk);
        check(out == 0, "latency: unchanged 3 clk after en");
        @(negedge clk);
        check(out == 0, "latency: unchanged 4 clk after en");
        @(negedge clk);
        check(out == 0, "latency: unchanged 5 clk after en");
        @(negedge clk);
        check(out == 32'sh0001_0000, "latency: valid exactly 6 clk after en");

        $display("== 11. en is a pulse, not a level ==");

        reset_dut();
        ki = 32'sh0001_0000;
        target = 32'sh0002_0000;
        actual = 0;
        one_tick();
        check(out == 32'sh0002_0000, "idle: after 1 tick -> 2.0");
        repeat (20) @(negedge clk);
        check(out == 32'sh0002_0000, "idle: 20 clk with en=0, output frozen");

        $display("== 12. sign handling with negative error ==");

        reset_dut();
        kp = 32'sh0001_0000;
        kd = 32'sh0001_0000;
        target = -32'sh0003_0000;
        actual = 0;
        one_tick();

        check(out == -32'sh0006_0000, "sign: P+D both negative -> -6.0");
        one_tick();

        check(out == -32'sh0003_0000, "sign: P only -> -3.0");

        $display("== 13. cascade: outer output rewrites inner target ==");

        reset_dut();
        kp = 32'sh0001_0000;  ki = 0;  kd = 0;
        target = 0;  actual = 32'sh0002_0000;
        one_tick();
        check(out == -32'sh0002_0000, "cascade: outer PI -> -2.0");

        target  = 32'sh000A_0000 - out;
        actual  = 32'sh000A_0000;
        one_tick();
        check(out == 32'sh0002_0000, "cascade: inner PD -> +2.0");

        $display("== 14. D-term low-pass (D_LPF=1, incomplete derivative) ==");

        reset_dut();
        kp = 0;  ki = 0;
        kd = -32'sh0001_0000;
        target = 0;
        out_max = BIG;  out_min = -BIG;

        actual = 32'sh0001_0000;
        one_tick();
        check    (out     == 32'sh0001_0000, "LPF: raw D reaches 65536 at once");
        check_lpf(out_lpf == 32'sh0000_8000, "LPF: y1 = 32768 (half of the step)");

        actual = 32'sh0002_0000;
        one_tick();
        check    (out     == 32'sh0001_0000, "LPF: raw D stays 65536");
        check_lpf(out_lpf == 32'sh0000_C000, "LPF: y2 = 49152");

        actual = 32'sh0003_0000;
        one_tick();
        check_lpf(out_lpf == 32'sh0000_E000, "LPF: y3 = 57344");

        actual = 32'sh0004_0000;
        one_tick();
        check_lpf(out_lpf == 32'sh0000_F000, "LPF: y4 = 61440");

        actual = 32'sh0005_0000;
        one_tick();
        check_lpf(out_lpf == 32'sh0000_F800, "LPF: y5 = 63488 (still rising, converging)");

        clear = 1'b1;
        one_tick();
        check_lpf(out_lpf == 0, "LPF: clear resets the filter to 0 (else 64512)");
        clear = 1'b0;

        reset_dut();
        en_dspan2 = 1'b1;
        kp = 0;  ki = 0;
        kd = -32'sh0001_0000;
        target = 0;

        actual = 32'sh0001_0000;
        one_tick();
        check(out == 32'sh0000_8000, "SPAN2: first tick is half (only e[k-1] exists)");

        actual = 32'sh0002_0000;
        one_tick();
        check(out == 32'sh0001_0000, "SPAN2: on a ramp, 2-point avg == 1-step diff");

        actual = 32'sh0003_0000;
        one_tick();
        check(out == 32'sh0001_0000, "SPAN2: holds on a ramp (KD_BAL needs no rescale)");

        clear = 1'b1;
        one_tick();
        check(out == 0, "SPAN2: clear makes err_d2 := e[k], d_err = 0 (else 65536)");
        clear = 1'b0;
        en_dspan2 = 1'b0;

        reset_dut();
        en_dspan4 = 1'b1;
        kp = 0;  ki = 0;
        kd = -32'sh0001_0000;
        target = 0;

        actual = 32'sh0001_0000;
        one_tick();
        check(out == 32'sh0000_4000, "SPAN4: tick1 = 1/4 (only e[k-1] in window)");

        actual = 32'sh0002_0000;
        one_tick();
        check(out == 32'sh0000_8000, "SPAN4: tick2 = 2/4");

        actual = 32'sh0003_0000;
        one_tick();
        check(out == 32'sh0000_C000, "SPAN4: tick3 = 3/4");

        actual = 32'sh0004_0000;
        one_tick();
        check(out == 32'sh0001_0000, "SPAN4: tick4 = 4/4, == 1-step diff");

        actual = 32'sh0005_0000;
        one_tick();
        check(out == 32'sh0001_0000, "SPAN4: holds on a ramp (KD_ARM needs no rescale)");

        clear = 1'b1;
        one_tick();
        check(out == 0, "SPAN4: clear resets err_d2/d3/d4 (else 65536)");
        clear = 1'b0;
        en_dspan4 = 1'b0;

        reset_dut();
        en_dlpf = 1'b1;  lpf_shift = 2'd0;
        kp = 0;  ki = 0;  kd = -32'sh0001_0000;  target = 0;
        actual = 32'sh0001_0000;
        one_tick();
        check(out == 32'sh0000_8000, "LPF s=0 (8/8): y1 = 32768");

        reset_dut();
        en_dlpf = 1'b1;  lpf_shift = 2'd1;
        kp = 0;  ki = 0;  kd = -32'sh0001_0000;  target = 0;
        actual = 32'sh0001_0000;
        one_tick();
        check(out == 32'sh0000_4000, "LPF s=1 (12/4): y1 = 16384");
        actual = 32'sh0002_0000;
        one_tick();
        check(out == 32'sh0000_7000, "LPF s=1 (12/4): y2 = 28672");
        lpf_shift = 2'd0;

        reset_dut();
        en_dlpf = 1'b1;  lpf_shift = 2'd2;
        kp = 0;  ki = 0;  kd = -32'sh0001_0000;  target = 0;
        actual = 32'sh0001_0000;
        one_tick();
        check(out == 32'sh0000_2000, "LPF s=2 (14/2): y1 = 8192");
        actual = 32'sh0002_0000;
        one_tick();
        check(out == 32'sh0000_3C00, "LPF s=2 (14/2): y2 = 15360");
        lpf_shift = 2'd0;

        reset_dut();
        en_dlpf = 1'b1;  lpf_shift = 2'd3;
        kp = 0;  ki = 0;  kd = -32'sh0001_0000;  target = 0;
        actual = 32'sh0001_0000;
        one_tick();
        check(out == 32'sh0000_1000, "LPF s=3 (15/1): y1 = 4096");
        actual = 32'sh0002_0000;
        one_tick();
        check(out == 32'sh0000_1F00, "LPF s=3 (15/1): y2 = 7936");

        repeat (300) begin
            actual = actual + 32'sh0001_0000;
            one_tick();
        end
        check((out >= 32'sh0000_FFF1) && (out <= 32'sh0001_0000),
              "LPF s=3: DC gain is 1, lands in [65521, 65536] (int floor)");
        lpf_shift = 2'd0;
        en_dlpf   = 1'b0;

        $display("");
        $display("================================");
        $display("  tb_pid_q16 checks = %0d, errors = %0d", checks, errors);
        if (errors == 0) $display("  *** PASS ***");
        else             $display("  *** FAIL ***");
        $display("================================");
        $finish;
    end

endmodule
