`timescale 1ns / 1ps

module tb_safety_guard;

    localparam integer CLKP = 20;

    localparam [15:0] CENTER = 16'd3095;

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

    reg        en    = 1'b0;
    reg        run   = 1'b0;
    reg [15:0] theta = 16'd0;
    reg signed [15:0] dpos = 16'sd0;

    wire safe_dut;
    wire safe_wide;

    safety_guard u_dut (
        .clk(clk), .rst_n(rst_n), .en(en), .run(run),
        .theta(theta), .center(CENTER), .dpos(dpos),
        .safe(safe_dut)
    );

    safety_guard #(
        .THETA_SPAN (16'd4000)
    ) u_wide (
        .clk(clk), .rst_n(rst_n), .en(en), .run(run),
        .theta(theta), .center(CENTER), .dpos(dpos),
        .safe(safe_wide)
    );

    task tick;
        input [15:0] th;
        input signed [15:0] dp;
        begin
            @(negedge clk);
            theta = th; dpos = dp; en = 1'b1;
            @(posedge clk); #1;
            @(negedge clk); en = 1'b0;
        end
    endtask

    task recover;
        integer k;
        begin
            for (k = 0; k < 12; k = k + 1) tick(CENTER, 16'sd0);
        end
    endtask

    integer i;

    initial begin
        $display("");
        $display("============== tb_safety_guard ==============");

        rst_n = 1'b0; run = 1'b0; en = 1'b0;
        theta = CENTER; dpos = 16'sd0;
        repeat (5) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        @(negedge clk);
        chk(safe_dut  === 1'b0, "after reset: safe must be 0");
        chk(safe_wide === 1'b0, "after reset: u_wide safe must be 0");

        @(negedge clk) run = 1'b1;

        tick(CENTER, 16'sd0);
        chk(safe_dut  === 1'b0, "A: 1st tick after reset must NOT report wrap (primed guard)");
        chk(safe_wide === 1'b0, "A: 1st tick on u_wide must NOT report wrap either");
        tick(CENTER, 16'sd0);
        chk(safe_dut === 1'b0, "A: steady at center -> no fault");

        tick(CENTER + 16'd800, 16'sd0);
        chk(safe_dut === 1'b0, "B: dev=800 must NOT trip (criterion is > THETA_SPAN)");

        tick(CENTER + 16'd801, 16'sd0);
        chk(safe_dut  === 1'b1, "B: dev=801 must trip fall");
        chk(safe_wide === 1'b0, "B: dev=801 must NOT trip on u_wide (SPAN=4000)");

        tick(CENTER - 16'd801, 16'sd0);
        chk(safe_dut === 1'b1, "B: dev=-801 must trip fall as well");
        recover;
        chk(safe_dut  === 1'b0, "B: back to center releases fall");
        chk(safe_wide === 1'b0, "B: u_wide safe after recover");

        tick(CENTER + 16'd801, 16'sd0);
        chk(safe_dut === 1'b1, "C: trip again");
        tick(CENTER + 16'd400, 16'sd0);
        repeat (8) tick(CENTER + 16'd400, 16'sd0);
        chk(safe_dut === 1'b1, "C: dev=400 must NOT release (criterion is < REL_SPAN)");
        tick(CENTER + 16'd399, 16'sd0);
        repeat (8) tick(CENTER + 16'd399, 16'sd0);
        chk(safe_dut === 1'b0, "C: dev=399 for 8 ticks must release");

        tick(CENTER, 16'sd3);
        chk(safe_dut === 1'b0, "D: |dp|=3 must NOT trip (criterion is > DPOS_MAX)");

        tick(CENTER, 16'sd4);
        chk(safe_dut  === 1'b1, "D: |dp|=4 must trip runaway");
        chk(safe_wide === 1'b1, "D: |dp|=4 must trip on u_wide too");

        tick(CENTER, 16'sd3);
        repeat (10) tick(CENTER, 16'sd3);
        chk(safe_dut  === 1'b1, "D: |dp|=3 must NOT release (needs <= DPOS_REL)");
        chk(safe_wide === 1'b1, "D: u_wide same");

        tick(CENTER, 16'sd1);
        repeat (8) tick(CENTER, 16'sd1);
        chk(safe_dut  === 1'b0, "D: |dp|=1 for 8 ticks must release");
        chk(safe_wide === 1'b0, "D: u_wide released as well");

        tick(CENTER, -16'sd4);
        chk(safe_dut === 1'b1, "D: |dp|=4 negative must trip");
        tick(CENTER, -16'sd1);
        repeat (8) tick(CENTER, -16'sd1);
        chk(safe_dut === 1'b0, "D: |dp|=1 negative must release");

        tick(CENTER + 16'd5, 16'sd0);
        chk(safe_wide === 1'b0, "E: near center, u_wide idle");

        tick(CENTER + 16'd1010, 16'sd0);
        chk(safe_wide === 1'b0, "E: dth=1010 must NOT trip wrap (criterion is > 1024)");
        chk(safe_dut  === 1'b1, "E: ...but fall DID trip on u_dut (dev=1010>800) - expected");

        recover;
        chk(safe_wide === 1'b0, "E: u_wide after recover");

        for (i = 3095; i <= 4167; i = i + 8) begin
            tick(i, 16'sd0);
        end
        chk(safe_wide === 1'b0, "E: smooth ramp up to 4167 must NOT trip wrap");

        tick(16'd135, 16'sd0);
        chk(safe_wide === 1'b1, "E: 4167 -> 135 (dth=4032) MUST trip wrap");

        repeat (20) tick(16'd135, 16'sd0);
        chk(safe_wide === 1'b1, "E: wrap stays latched while far from center");

        for (i = 135; i <= 3095; i = i + 16) begin
            tick(i, 16'sd0);
        end
        chk(safe_wide === 1'b0, "E: rotating back near center must release wrap");

        tick(CENTER + 16'd900, 16'sd0);
        chk(safe_dut === 1'b1, "F: fall trips (precondition)");

        @(negedge clk) run = 1'b0;
        tick(CENTER + 16'd900, 16'sd0);
        chk(safe_dut === 1'b0, "F: run=0 must clear safe even while fallen");

        @(negedge clk) run = 1'b1;
        tick(CENTER + 16'd900, 16'sd0);
        chk(safe_dut === 1'b1, "F: re-entering closed loop while fallen must trip again");

        recover;
        chk(safe_dut === 1'b0, "F: final recover");
        tick(CENTER+16'd801,16'sd0);
        repeat(7) tick(CENTER,16'sd0);
        tick(CENTER+16'd400,16'sd0);
        chk(safe_dut === 1'b1,"G: invalid eighth recovery sample must not release");
        recover;
        chk(safe_dut === 1'b0,"G: eight genuinely valid samples release");

        $display("");
        $display("=============================================");
        if (errors == 0) $display("  ALL PASS: %0d checks, 0 errors", checks);
        else             $display("  FAILED: %0d checks, %0d errors", checks, errors);
        $display("=============================================");
        $display("");
        $finish;
    end

endmodule
