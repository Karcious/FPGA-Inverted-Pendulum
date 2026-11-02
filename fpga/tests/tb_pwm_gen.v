`timescale 1ns / 1ps

module tb_pwm_gen;

    localparam integer CLKP  = 20;
    localparam integer PERIOD = 2500;

    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #(CLKP/2) clk = ~clk;

    integer errors = 0;
    integer checks = 0;
    task chk;
        input         cond;
        input [255:0] msg;
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $display("  [FAIL] %0s", msg);
            end
        end
    endtask

    reg signed [15:0] duty_in   = 16'sd0;
    reg        [15:0] duty_max  = 16'd0;
    reg        [15:0] duty_min  = 16'd0;
    reg        [3:0] comp_shift = 0;

    wire              pwm, an1, an2, blanking;
    wire signed [15:0] duty_applied;

    pwm_gen #(.CLK_FREQ(50_000_000), .PWM_HZ(20_000), .DSH(10), .COMP_SHAPING(1)) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .duty_in      (duty_in),
        .duty_max     (duty_max),
        .duty_min     (duty_min),
        .comp_shift   (comp_shift),
        .pwm          (pwm),
        .an1          (an1),
        .an2          (an2),
        .duty_applied (duty_applied),
        .blanking     (blanking)
    );

    integer hi, k, p, bad, bl_len, bl_n, bl_bad;

    task count_high;
        output integer cnt;
        input  integer n;
        begin
            cnt = 0;
            for (k = 0; k < n; k = k + 1) begin
                @(posedge clk); #1;
                if (pwm) cnt = cnt + 1;
            end
        end
    endtask

    task meas_period;
        output integer per;
        begin
            while (pwm)  begin @(posedge clk); #1; end
            while (!pwm) begin @(posedge clk); #1; end
            per = 0;
            while (pwm)  begin @(posedge clk); #1; per = per + 1; end
            while (!pwm) begin @(posedge clk); #1; per = per + 1; end
        end
    endtask

    task settle;
        begin
            repeat (4) @(posedge clk);
            repeat (PERIOD * 3) @(posedge clk);
        end
    endtask

    task catch_blank;
        output integer len;
        output integer viol;
        begin
            bl_n = 0;
            while (!blanking && bl_n < PERIOD * 8) begin
                @(posedge clk); #1; bl_n = bl_n + 1;
            end
            len   = 0;
            viol  = 0;
            while (blanking && len < PERIOD * 8) begin

                if (pwm || an1 || an2 || duty_applied!=0) viol = viol + 1;
                @(posedge clk); #1;
                len = len + 1;
            end
        end
    endtask

    task ramp_case;
        input integer raw;
        input integer eapp;
        input integer ehi;
        input [255:0] lbl;
        begin
            @(negedge clk) duty_in = raw;
            settle();
            count_high(hi, PERIOD * 4);
            $display("  raw=%0d  applied=%0d (expect %0d)  high=%0d (expect %0d)  %0s",
                     raw, duty_applied, eapp, hi, ehi, lbl);
            chk(duty_applied === eapp[15:0], lbl);
            chk(hi === ehi, "ramp shape: PWM high count must follow applied");
        end
    endtask

    initial begin
        $display("");
        $display("============== tb_pwm_gen ==============");

        rst_n   = 1'b0;
        duty_in = 16'sd0;
        repeat (10) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        repeat (10) @(posedge clk);

        $display("");
        $display("--- 4. duty = 0 (coast) ---");
        settle();
        count_high(hi, PERIOD * 4);
        $display("  pwm=%0d an1=%0d an2=%0d  high=%0d (expect 0 0 0 0)",
                 pwm, an1, an2, hi);
        chk(pwm === 1'b0 && an1 === 1'b0 && an2 === 1'b0,
            "duty=0: pwm low, an1=an2=0 (coast)");
        chk(hi === 0, "duty=0: no high ticks at all");

        $display("");
        $display("--- 2. forward duty = 256 (25%%) ---");
        @(negedge clk) duty_in = 16'sd256;
        settle();
        count_high(hi, PERIOD * 4);
        $display("  an1=%0d an2=%0d  high=%0d (expect 1 0 2500)", an1, an2, hi);
        $display("  duty_applied=%0d (expect 256)", duty_applied);
        chk(an1 === 1'b1 && an2 === 1'b0, "positive duty -> an1=1, an2=0");
        chk(hi === 2500, "duty_in=256 -> exactly 25%% (2500/10000)");
        chk(duty_applied === 16'sd256, "duty_applied = 256 (no limit, no deadzone)");

        $display("");
        $display("--- 2b. forward duty = 512 (50%%) ---");
        @(negedge clk) duty_in = 16'sd512;
        settle();
        count_high(hi, PERIOD * 4);
        $display("  high=%0d (expect 5000)", hi);
        chk(hi === 5000, "duty_in=512 -> exactly 50%%");

        $display("");
        $display("--- 1. carrier period ---");
        meas_period(p);
        $display("  pwm period = %0d clocks (expect %0d -> 20kHz)", p, PERIOD);
        chk(p === PERIOD, "carrier is exactly 50MHz/2500 = 20kHz");

        $display("");
        $display("--- 3. reverse duty = -256 ---");
        @(negedge clk) duty_in = -16'sd256;
        settle();
        count_high(hi, PERIOD * 4);
        $display("  an1=%0d an2=%0d  high=%0d (expect 0 1 2500)", an1, an2, hi);
        $display("  duty_applied=%0d (expect -256)", duty_applied);
        chk(an1 === 1'b0 && an2 === 1'b1, "negative duty -> an1=0, an2=1");
        chk(hi === 2500, "duty_in=-256 -> same magnitude as +256");
        chk(duty_applied === -16'sd256, "duty_applied = -256");

        $display("");
        $display("--- 5. saturation: duty_in = 30000 ---");
        @(negedge clk) duty_in = 16'sd30000;
        settle();
        count_high(hi, PERIOD * 4);
        $display("  duty_applied=%0d (expect 1024)", duty_applied);
        $display("  high=%0d (expect %0d = 100%%)", hi, PERIOD * 4);
        chk(duty_applied === 16'sd1024, "over-range clamps to full scale 1024");
        chk(hi === PERIOD * 4, "full scale -> 100%% duty (official code caps at 94.4%%)");
        chk(an1 === 1'b1 && an2 === 1'b0, "clamped positive still forward");

        $display("");
        $display("--- 6. duty_in = -32768 (abs overflow trap) ---");
        @(negedge clk) duty_in = -16'sd32768;
        settle();
        $display("  duty_applied=%0d (expect -1024)", duty_applied);
        $display("  an1=%0d an2=%0d (expect 0 1 = reverse)", an1, an2);
        chk(duty_applied === -16'sd1024, "-32768 must clamp to -1024, NOT overflow to +");
        chk(an1 === 1'b0 && an2 === 1'b1, "-32768 must stay reverse");

        $display("");
        $display("--- 7a. duty_max = 512 ---");
        @(negedge clk) duty_in = 16'sd1024;
        @(negedge clk) duty_max = 16'd512;
        settle();
        count_high(hi, PERIOD * 4);
        $display("  duty_applied=%0d (expect 512)", duty_applied);
        $display("  high=%0d (expect 5000 = 50%%)", hi);
        chk(duty_applied === 16'sd512, "duty_max=512 limits full-scale input to 512");
        chk(hi === 5000, "duty_max=512 -> 50%% duty");

        $display("");
        $display("--- 7b. duty_min = 100, duty_in = 3 (RAMP: comp = min(|raw|,dmin)) ---");
        @(negedge clk) duty_max = 16'd0;
        @(negedge clk) duty_min = 16'd100;
        @(negedge clk) duty_in  = 16'sd3;
        settle();
        count_high(hi, PERIOD * 4);

        $display("  duty_applied=%0d (expect 6 = 3 + min(3,100))", duty_applied);
        $display("  high=%0d (expect 56)", hi);
        chk(duty_applied === 16'sd6,
            "RAMP: |raw|=3 with dmin=100 -> 6, NOT 103 (v1.08) and NOT 100 (floor)");
        chk(hi === 56, "ramp compensation gives exactly 56/10000 at |raw|=3");

        $display("");
        $display("--- 7b-2. duty_in = -3, duty_min = 100 -> must be -6 ---");
        @(negedge clk) duty_in = -16'sd3;
        settle();
        count_high(hi, PERIOD * 4);
        $display("  duty_applied=%0d (expect -6)  an1=%0d an2=%0d (expect 0 1)",
                 duty_applied, an1, an2);
        chk(duty_applied === -16'sd6, "ramp compensation must follow the sign of raw");
        chk(an1 === 1'b0 && an2 === 1'b1, "negative compensation still reverse");

        $display("");
        $display("--- 7b-3. duty_max=512, duty_min=100, duty_in=1024 -> stay 512 ---");
        @(negedge clk) duty_max = 16'd512;
        @(negedge clk) duty_in  = 16'sd1024;
        settle();
        count_high(hi, PERIOD * 4);
        $display("  duty_applied=%0d (expect 512, NOT 612)  high=%0d (expect 5000)",
                 duty_applied, hi);
        chk(duty_applied === 16'sd512,
            "compensation must NOT break through duty_max");
        chk(hi === 5000, "limit still exactly 50%% duty after compensation");
        @(negedge clk) duty_max = 16'd0;

        $display("");
        $display("--- 7c. duty_min set, but duty_in = 0 -> must STAY 0 ---");
        @(negedge clk) duty_in = 16'sd0;
        settle();
        count_high(hi, PERIOD * 4);
        $display("  duty_applied=%0d (expect 0)  high=%0d (expect 0)",
                 duty_applied, hi);
        chk(duty_applied === 16'sd0,
            "exact zero must NOT be boosted (otherwise the pendulum never settles)");
        chk(hi === 0, "exact zero -> no drive");

        $display("");
        $display("--- 7d. RAMP SHAPE: min(|raw|,dmin), dmin = 100, sweep ---");
        ramp_case(16'sd0,   16'sd0,   0,    "zero stays zero");
        ramp_case(16'sd25,  16'sd50,  488,  "RAMP not STEP (v1.08 gave 125)");
        ramp_case(16'sd50,  16'sd100, 976,  "half compensation point");
        ramp_case(16'sd99,  16'sd198, 1932, "just below the knee");
        ramp_case(16'sd100, 16'sd200, 1952, "knee: ramp meets plateau");
        ramp_case(16'sd150, 16'sd250, 2440, "identical to v1.08");

        $display("");
        $display("--- 8. direction change blanking ---");
        @(negedge clk) duty_min = 16'd0;

        @(negedge clk) duty_in  = 16'sd1024;
        settle();
        chk(an1 === 1'b1 && an2 === 1'b0, "settled forward before the flip");
        chk(pwm === 1'b1, "100%% duty -> pwm is high right before the flip");

        @(negedge clk) duty_in = -16'sd1024;
        @(posedge clk); #1;
        $display("  clock right after the flip request: pwm=%0d an1=%0d an2=%0d",
                 pwm, an1, an2);
        chk(pwm === 1'b0 && an1 === 1'b0 && an2 === 1'b0,
            "drive must go off on the very clock the flip is requested");

        catch_blank(bl_len, bl_bad);
        $display("  blanking lasted %0d clocks (expect ~%0d)", bl_len, PERIOD);
        $display("  violations during blank (pwm/an1/an2 not all 0): %0d", bl_bad);
        chk(bl_len >= PERIOD - 4 && bl_len <= PERIOD + 4,
            "direction change coasts for exactly one PWM period");
        chk(bl_bad === 0,
            "an1=an2=0 and pwm=0 for the whole blanking window (no shoot-through)");

        settle();
        $display("  after blanking: an1=%0d an2=%0d (expect 0 1)", an1, an2);
        chk(an1 === 1'b0 && an2 === 1'b1, "reverse applied only after blanking");

        $display("");
        @(negedge clk);comp_shift=1;#1;
        chk(dut.shape_shift_q==0,"shape parameter changes only on clock edge");
        @(posedge clk);#1;
        chk(dut.shape_shift_q==1,"shape parameter has one registered stage");
        duty_min=130; duty_in=10; settle();
        chk(duty_applied==30,"static shaped compensation: 10+20");
        comp_shift=15;settle();
        chk(dut.shape_shift_q==3 && duty_applied==90,"shape shift saturates at 3");
        comp_shift=1;
        duty_in=70;settle();chk(duty_applied==200,"static compensation capped at 130");
        duty_min=50;settle();chk(duty_applied==120,"moving compensation capped at 50");
        duty_in=-70;settle();chk(duty_applied==-120,"shaped compensation symmetric");

        duty_in=10;duty_min=130;settle();
        for(k=0;k<8;k=k+1)begin
            @(negedge clk);duty_in=20+10*k;
            @(posedge clk);#1;
            chk(duty_applied==(k<3 ? 30 : 3*(20+10*(k-3))),
                "four-stage duty/sign/compensation alignment");
        end
        duty_in=0;repeat(6)@(posedge clk);#1;
        chk(duty_applied==0 && pwm==0,"zero stops shaped compensation");
        $display("=============================================");
        if (errors == 0) $display("  ALL PASS: %0d checks, 0 errors", checks);
        else             $display("  FAILED: %0d checks, %0d errors", checks, errors);
        $display("=============================================");
        $display("");
        $finish;
    end

endmodule
