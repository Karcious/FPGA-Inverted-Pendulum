`timescale 1ns / 1ps

module tb_adc_capture;

    localparam integer CLKP = 20;

    localparam integer OSR  = 256;
    localparam integer BLKP = OSR * 40;

    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #(CLKP/2) clk = ~clk;

    wire        adc_clk;
    wire        adc_oe_n;
    wire [9:0]  adc_data_in;
    wire [11:0] theta_raw;
    wire        adc_valid;
    wire        otr_seen;

    reg         adc_otr = 1'b0;

    adc_capture u_dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .adc_clk     (adc_clk),
        .adc_oe_n    (adc_oe_n),
        .adc_data_in (adc_data_in),
        .adc_otr     (adc_otr),
        .theta_raw   (theta_raw),
        .adc_valid   (adc_valid),
        .otr_seen    (otr_seen)
    );

    integer   t_pd_ns   = 12;
    integer   pat_const = 500;
    integer   pat_ramp  = 0;
    integer   ramp_mod  = 16;
    integer   ramp_cnt  = 0;

    wire [9:0] code_src = pat_ramp ? (ramp_cnt % ramp_mod) : pat_const;

    reg  [9:0] adc_launch = 10'd0;
    reg  [9:0] adc_dout   = 10'd0;

    always @(posedge adc_clk) begin
        ramp_cnt   <= ramp_cnt + 1;
        adc_launch <= code_src;
    end

    always @(adc_launch) adc_dout <= #(t_pd_ns) adc_launch;

    assign adc_data_in = adc_dout;

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

    reg [11:0] last_avg;
    always @(posedge clk) if (adc_valid) last_avg <= theta_raw;

    task wait_blocks;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) @(posedge adc_valid);
            @(posedge clk);
        end
    endtask

    integer vcnt = 0;
    time    t_first = 0, t_last = 0;
    always @(posedge clk) if (adc_valid) begin
        vcnt = vcnt + 1;
        if (vcnt == 1) t_first = $time;
        t_last = $time;
    end

    integer i;
    integer bad;
    integer pd;
    integer margin_dut, margin_alt;
    time    t0, t1;

    initial begin
        $display("");
        $display("==================== tb_adc_capture ====================");

        rst_n = 1'b0;
        #200;
        rst_n = 1'b1;
        #200;

        chk(adc_oe_n === 1'b0, "adc_oe_n must stay 0 (active low, always enabled)");

        @(posedge adc_clk) t0 = $time;
        repeat (10) @(posedge adc_clk);
        t1 = $time;
        $display("  adc_clk period = %0d ns -> 25MHz", (t1 - t0)/10);
        chk((t1 - t0) == 10*40, "adc_clk period must be 40ns (25MHz)");

        $display("");
        $display("--- (1) constant 500, expect theta_raw = 2000 ---");
        pat_ramp  = 0;
        pat_const = 500;
        repeat (3) @(posedge adc_valid);
        wait_blocks(1);
        $display("  theta_raw = %0d", last_avg);
        chk(last_avg == 12'd2000, "constant 500 -> average must be 2000");

        $display("");
        $display("--- (1b) full scale 1023, expect theta_raw = 4092 ---");
        pat_ramp  = 0;
        pat_const = 1023;
        repeat (3) @(posedge adc_valid);
        wait_blocks(1);
        $display("  theta_raw = %0d", last_avg);
        chk(last_avg == 12'd4092,
            "full scale 1023 -> 4092 (accumulator must hold 256*1023)");

        $display("");
        $display("--- (2) ramp 0..15 cyclic, expect theta_raw = 30 always ---");
        pat_ramp = 1;
        ramp_mod = 16;
        repeat (3) @(posedge adc_valid);
        bad = 0;
        for (i = 0; i < 20; i = i + 1) begin
            wait_blocks(1);
            if (last_avg !== 12'd30) begin
                bad = bad + 1;
                if (bad <= 3) $display("  block %0d = %0d (expect 30)", i, last_avg);
            end
        end
        $display("  20 blocks checked, %0d block(s) deviated", bad);
        chk(bad == 0, "every OSR-sample window must be a whole number of ramp cycles");

        $display("");
        $display("--- (3) adc_valid period ---");
        vcnt = 0;
        @(posedge clk);
        while (vcnt < 11) @(posedge clk);
        $display("  11 pulses span %0d ns -> period %0d ns (expect %0d ns)",
                 t_last - t_first, (t_last - t_first)/10, BLKP);
        chk((t_last - t_first) == 10*BLKP,
            "adc_valid period must be OSR x 40ns");

        $display("");
        $display("--- (4) OTR latch ---");
        chk(otr_seen === 1'b0, "otr_seen must start at 0");
        @(posedge clk) adc_otr = 1'b1;
        repeat (4) @(posedge clk);
        adc_otr = 1'b0;
        repeat (4) @(posedge clk);
        chk(otr_seen === 1'b1, "otr_seen must latch to 1 after one adc_otr");

        $display("");
        $display("--- (5) T_PD sweep (ADC output delay) ---");
        $display("     DUT point      = the clk edge where adc_clk rises");
        $display("                      (i.e. 20ns after the previous rise)");
        $display("     official-style = the clk edge 10ns after the rise");
        $display("");
        $display("     T_PD     DUT margin   official margin   func");
        pat_ramp  = 0;
        pat_const = 500;
        for (pd = 2; pd <= 19; pd = pd + 2) begin
            t_pd_ns    = pd;
            margin_dut = 20 - pd;
            margin_alt = 10 - pd;
            #2000;
            repeat (4) @(posedge adc_valid);
            wait_blocks(1);
            $display("     %0d ns    +%0d ns        %0d ns             %0s",
                     pd, margin_dut, margin_alt,
                     (last_avg == 12'd2000) ? "OK" : "BAD");
            chk(last_avg == 12'd2000,
                "theta_raw must still be 2000 at this T_PD");
            chk(margin_dut > 0, "DUT sampling margin must be positive");
        end

        $display("");
        $display("=======================================================");
        if (errors == 0)
            $display("  ALL PASS: %0d checks, 0 errors", checks);
        else
            $display("  FAILED: %0d checks, %0d errors", checks, errors);
        $display("=======================================================");
        $display("");
        $finish;
    end

endmodule
