`timescale 1ns / 1ps

module tb_encoder_decode;

    localparam integer CLKP       = 20;
    localparam integer CW         = 32;
    localparam integer POS_INIT   = 10000;
    localparam integer HOLD       = 400;

    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #(CLKP/2) clk = ~clk;

    integer errors = 0;
    integer checks = 0;
    integer base   = 0;
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

    integer qidx = 0;
    wire a = (qidx == 1) || (qidx == 2);
    wire b = (qidx == 2) || (qidx == 3);

    task qgo;
        input integer n;
        integer k;
        begin
            for (k = 0; k < ((n > 0) ? n : -n); k = k + 1)
                begin
                    qidx = (n > 0) ? ((qidx + 1) & 3) : ((qidx + 3) & 3);
                    #(HOLD);
                end
            #(HOLD);
        end
    endtask

    task qjump;
        input integer t;
        begin
            qidx = t;
            #(HOLD);
        end
    endtask

    reg               sample_en = 1'b0;

    wire signed [CW-1:0] pos;
    wire signed [15:0]   delta;
    wire                 dir, fault_tick, fault_seen;

    encoder_decode #(.CW(CW), .POS_INIT(POS_INIT)) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .enc_a      (a),
        .enc_b      (b),
        .sample_en  (sample_en),
        .pos        (pos),
        .delta      (delta),
        .dir        (dir),
        .fault_tick (fault_tick),
        .fault_seen (fault_seen)
    );

    task tick;
        begin
            @(negedge clk) sample_en = 1'b1;
            @(negedge clk) sample_en = 1'b0;
            @(posedge clk); #1;
        end
    endtask

    initial begin
        $display("");
        $display("============== tb_encoder_decode ==============");

        qidx  = 3;
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        repeat (10) @(posedge clk); #1;

        $display("--- 7. reset with encoder parked at 01 (qidx=3) ---");
        $display("  pos = %0d (expect %0d)", pos, POS_INIT);
        chk(pos === POS_INIT,
            "no spurious count at reset when parked at 01 (official encoder.v fails this)");
        chk(fault_seen === 1'b0, "no fault during warm-up");

        $display("");
        $display("--- 1. reset value ---");
        chk(pos === POS_INIT, "pos initialises to POS_INIT");

        qgo(10);
        $display("");
        $display("--- 2. forward 10 counts ---");
        $display("  pos = %0d (expect %0d), dir = %0d", pos, POS_INIT + 10, dir);
        chk(pos === POS_INIT + 10, "forward 10 -> pos + 10");
        chk(dir === 1'b1, "dir = 1 (forward)");

        qgo(-3);
        $display("");
        $display("--- 3. reverse 3 counts ---");
        $display("  pos = %0d (expect %0d), dir = %0d", pos, POS_INIT + 7, dir);
        chk(pos === POS_INIT + 7, "reverse 3 -> pos - 3");
        chk(dir === 1'b0, "dir = 0 (reverse)");

        $display("");
        $display("--- 4. delta per sample tick ---");
        tick();
        qgo(5);
        tick();
        $display("  after 5 counts,  delta = %0d (expect 5)",  delta);
        chk(delta === 16'sd5, "delta = +5 after 5 forward counts");

        tick();
        $display("  idle tick,       delta = %0d (expect 0)",  delta);
        chk(delta === 16'sd0, "delta = 0 when idle");

        qgo(-4);
        tick();
        $display("");
        $display("--- 5. negative delta ---");
        $display("  after 4 reverse counts, delta = %0d (expect -4)", delta);
        chk(delta === -16'sd4, "delta = -4 after 4 reverse counts");

        $display("");
        $display("--- 6. missed step (A and B change together) ---");

        tick();
        qjump(0);
        tick();
        base = pos;
        qjump(2);
        tick();
        $display("  base = %0d, after illegal jump pos = %0d (must be equal)",
                 base, pos);
        $display("  fault_tick = %0d, fault_seen = %0d (both expect 1)",
                 fault_tick, fault_seen);
        chk(pos === base, "missed step must not move pos (direction unknown)");
        chk(fault_tick === 1'b1, "missed step raises fault_tick");
        chk(fault_seen === 1'b1, "missed step latches fault_seen");

        tick();
        $display("  next tick, fault_tick = %0d (expect 0), fault_seen = %0d (expect 1)",
                 fault_tick, fault_seen);
        chk(fault_tick === 1'b0, "fault_tick is per-tick, clears on next tick");
        chk(fault_seen === 1'b1, "fault_seen stays latched");

        $display("");
        $display("=============================================");
        if (errors == 0) $display("  ALL PASS: %0d checks, 0 errors", checks);
        else             $display("  FAILED: %0d checks, %0d errors", checks, errors);
        $display("=============================================");
        $display("");
        $finish;
    end

endmodule
