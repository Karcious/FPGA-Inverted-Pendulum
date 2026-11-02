`timescale 1ns / 1ps

module tb_capture_decim;

    localparam integer CLKP  = 20;
    localparam integer DEPTH = 16;
    localparam integer AW    = 4;

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

    reg        arm  = 1'b0;
    reg [2:0]  sub  = 3'd0;
    reg [15:0] tval = 16'd100;
    wire       tick = (sub == 3'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)    sub <= 3'd0;
        else if (arm)  sub <= 3'd0;
        else           sub <= sub + 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)    tval <= 16'd100;
        else if (arm)  tval <= 16'd100;
        else if (tick) tval <= tval + 1'b1;
    end

    wire          done_keep, done_dec;
    wire [AW-1:0] idx_keep, idx_dec;

    reg  [AW-1:0] rd_addr = {AW{1'b0}};
    wire [15:0]   rd_keep, rd_dec;

    capture #(.DEPTH(DEPTH), .AW(AW), .NCH(1), .DW(16),
              .DECIM(1), .DCW(1)) u_keep (
        .clk(clk), .rst_n(rst_n),
        .sample_en(tick), .sample_data(tval),
        .trig_ch(2'd0), .trig_level(16'sd0), .trig_edge(2'd2), .pre_len({AW{1'b0}}),
        .arm(arm), .disarm(1'b0),
        .armed(), .done(done_keep), .trig_idx(idx_keep),
        .rd_addr(rd_addr), .rd_data(rd_keep)
    );

    capture #(.DEPTH(DEPTH), .AW(AW), .NCH(1), .DW(16),
              .DECIM(4), .DCW(2)) u_dec (
        .clk(clk), .rst_n(rst_n),
        .sample_en(tick), .sample_data(tval),
        .trig_ch(2'd0), .trig_level(16'sd0), .trig_edge(2'd2), .pre_len({AW{1'b0}}),
        .arm(arm), .disarm(1'b0),
        .armed(), .done(done_dec), .trig_idx(idx_dec),
        .rd_addr(rd_addr), .rd_data(rd_dec)
    );

    integer i;
    integer bad;
    integer frozen_keep, frozen_dec;
    integer got_keep [0:DEPTH-1];
    integer got_dec  [0:DEPTH-1];

    initial begin
        $display("");
        $display("============== tb_capture_decim ==============");

        rst_n = 1'b0;
        arm   = 1'b0;
        repeat (5) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        @(negedge clk) arm   = 1'b1;
        @(negedge clk) arm   = 1'b0;

        wait (done_keep === 1'b1);
        wait (done_dec  === 1'b1);
        repeat (4) @(posedge clk);

        $display("  trig_idx  keep = %0d   dec = %0d", idx_keep, idx_dec);
        chk(idx_keep === 4'd1, "DECIM=1: trig_idx must be 1 (2nd stored sample)");
        chk(idx_dec  === 4'd1, "DECIM=4: trig_idx must be 1 (2nd stored sample)");

        for (i = 0; i < DEPTH; i = i + 1) begin
            @(negedge clk) rd_addr = i;
            @(posedge clk); #1;
            got_keep[i] = $signed(rd_keep);
            got_dec[i]  = $signed(rd_dec);
        end

        $display("");
        $display("--- DECIM=1: expect logical[i] = 101 + i ---");
        bad = 0;
        for (i = 0; i < DEPTH; i = i + 1) begin
            if (got_keep[i] !== 101 + i) begin
                if (bad < 4) $display("  [%0d] got %0d, expect %0d", i, got_keep[i], 101 + i);
                bad = bad + 1;
            end
        end
        $display("  mismatches: %0d / %0d", bad, DEPTH);
        chk(bad == 0, "DECIM=1 must be identical to no-decimation");

        $display("");
        $display("--- DECIM=4: expect logical[i] = 107 + 4*i ---");
        bad = 0;
        for (i = 0; i < DEPTH; i = i + 1) begin
            if (got_dec[i] !== 107 + 4*i) begin
                if (bad < 4) $display("  [%0d] got %0d, expect %0d", i, got_dec[i], 107 + 4*i);
                bad = bad + 1;
            end
        end
        $display("  mismatches: %0d / %0d", bad, DEPTH);
        chk(bad == 0, "DECIM=4 must store exactly every 4th sample");

        repeat (200) @(posedge tick);
        @(negedge clk) rd_addr = {AW{1'b0}};
        @(posedge clk); #1;
        frozen_keep = $signed(rd_keep);
        frozen_dec  = $signed(rd_dec);
        $display("");
        $display("--- after 200 more ticks (done must freeze the buffer) ---");
        $display("  keep logical[0] = %0d (expect 101)", frozen_keep);
        $display("  dec  logical[0] = %0d (expect 107)", frozen_dec);
        chk(frozen_keep === 101, "DECIM=1 buffer must be frozen after done");
        chk(frozen_dec  === 107, "DECIM=4 buffer must be frozen after done");

        $display("");
        $display("=============================================");
        if (errors == 0) $display("  ALL PASS: %0d checks, 0 errors", checks);
        else             $display("  FAILED: %0d checks, %0d errors", checks, errors);
        $display("=============================================");
        $display("");
        $finish;
    end

endmodule
