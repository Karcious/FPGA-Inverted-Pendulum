`timescale 1ns/1ps
module tb_fuzzy_pid_q16;
    localparam signed [31:0] Q=65536, BIG=32'sh40000000;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,en=0,clear=0,fuzzy_enable=1;
    reg signed [31:0] target=0,actual=0,kp=16*Q,ki=-Q/100,kd=16*Q;
    wire signed [31:0] out,fixed_out,outer_out,kp_eff,ki_eff,kd_eff;
    wire signed [31:0] outer_kp,outer_ki,outer_kd;
    wire [9:0] pscale,dscale;
    wire signed [32:0] filtered;
    integer checks=0,n,old_p,old_d;
    reg compare_fixed=0,compare_outer=0;
    reg [9:0] p_positive,d_positive;
    reg signed [32:0] filter_before;

    fuzzy_pid_q16 dut(.clk(clk),.rst_n(rst_n),.en(en),.clear(clear),
        .fuzzy_enable(fuzzy_enable),.target(target),.actual(actual),.kp(kp),.ki(ki),.kd(kd),
        .out_max(BIG),.out_min(-BIG),.en_dlpf(1'b1),.en_dspan2(1'b1),
        .en_dspan4(1'b0),.lpf_shift(2'd0),.out(out),.kp_eff(kp_eff),.ki_eff(ki_eff),
        .kd_eff(kd_eff),.kp_scale_dbg(pscale),.kd_scale_dbg(dscale),.ec_filtered_dbg(filtered));
    fuzzy_pid_q16 #(.TUNE_ENABLE(0)) outer_bypass(
        .clk(clk),.rst_n(rst_n),.en(en),.clear(clear),.fuzzy_enable(fuzzy_enable),
        .target(target),.actual(actual),.kp(kp),.ki(ki),.kd(kd),.out_max(BIG),.out_min(-BIG),
        .en_dlpf(1'b1),.en_dspan2(1'b1),.en_dspan4(1'b0),.lpf_shift(2'd0),
        .out(outer_out),.kp_eff(outer_kp),.ki_eff(outer_ki),.kd_eff(outer_kd));
    pid_q16 reference_fixed(.clk(clk),.rst_n(rst_n),.en(en),.clear(clear),
        .target(target),.actual(actual),.kp(kp),.ki(ki),.kd(kd),.out_max(BIG),.out_min(-BIG),
        .en_dlpf(1'b1),.en_dspan2(1'b1),.en_dspan4(1'b0),.lpf_shift(2'd0),.out(fixed_out));

    task check; input ok; input [8*96-1:0] name; begin
        checks=checks+1;
        if(ok!==1) $fatal(1,"%0s P/D=%0d/%0d gains=%0d/%0d/%0d ec=%0d",
            name,pscale,dscale,kp_eff,ki_eff,kd_eff,filtered);
    end endtask
    task sample; input signed [31:0] e; input do_clear; begin
        old_p=pscale;old_d=dscale;
        @(negedge clk);target=e;clear=do_clear;en=1;
        @(negedge clk);en=0;clear=0;repeat(40)@(negedge clk);#1;
        if(fuzzy_enable) begin
            check(pscale>=256 && pscale<=288,"P multiplier stays in 1..1.125");
            check(dscale>=256 && dscale<=320,"D multiplier stays in 1..1.25");
            check(ki_eff===ki,"Ki always equals signed runtime baseline");
            if(!do_clear) begin
                check($signed({1'b0,pscale})-old_p<=1 && old_p-$signed({1'b0,pscale})<=1,
                    "P multiplier slew is at most one Q8 count per sample");
                check($signed({1'b0,dscale})-old_d<=2 && old_d-$signed({1'b0,dscale})<=2,
                    "D multiplier slew is at most two Q8 counts per sample");
            end
        end
    end endtask

    always @(posedge clk) begin
        #1;
        if(rst_n && compare_fixed) check(out===fixed_out,"fixed bypass is cycle-identical");
        if(rst_n && compare_outer) check(outer_out===fixed_out,"outer tuning disabled is cycle-identical");
    end

    initial begin
        repeat(4)@(negedge clk);rst_n=1;
        sample(32*Q,0);
        check(pscale==256 && dscale==256 && filtered==0,
            "first sample without explicit clear seeds history at nominal gains");
        check(kp_eff===kp && ki_eff===ki && kd_eff===kd,
            "first un-cleared sample preserves original runtime baseline");
        sample(0,1);
        check(pscale==256 && dscale==256 && filtered==0,"clear restores nominal scheduling state");
        check(kp_eff===kp && kd_eff===kd,"clear preserves signed baseline gains");
        for(n=0;n<100;n=n+1) begin
            sample((n%2 ? -Q : Q),0);
            check(pscale==256 && dscale==256,"one-code measurement jitter stays on nominal platform");
        end
        sample(0,1);sample(4*Q,0);
        check(filtered==Q,"ec filter uses one quarter of raw error difference");
        filter_before=filtered;repeat(100)@(negedge clk);#1;
        check(filtered===filter_before,"ec filter updates only at control samples");
        for(n=2;n<=90;n=n+1) sample(n*4*Q,0);
        check(pscale==288,"continued outward motion reaches bounded P maximum");
        check(dscale>256,"continued fast motion increases derivative damping");

        sample(48*Q,1);
        for(n=11;n>=0;n=n-1) begin
            sample(n*4*Q,0);
            check(pscale==256,"approaching vertical does not increase proportional gain");
        end
        check(target==0 && filtered<0 && dscale>256,"motion through zero keeps additional damping");

        sample(0,1);sample(16*Q,0);p_positive=pscale;d_positive=dscale;
        sample(0,1);sample(-16*Q,0);
        check(pscale==p_positive && dscale==d_positive,"positive and negative outward motion are symmetric");
        kp=32'sh7fffffff;kd=32'sh80000000;ki=32'sh7fffffff;
        sample(0,1);for(n=1;n<=60;n=n+1)sample(n*4*Q,0);
        check(kp_eff==32'sh7fffffff && kd_eff==32'sh80000000,
            "extreme signed gains saturate without wraparound");
        check(ki_eff==32'sh7fffffff,"integral gain passes through without scaling overflow");

        kp=-187*Q-Q/2;kd=0;ki=-24*Q;
        sample(0,1);for(n=1;n<=60;n=n+1)sample(n*4*Q,0);
        check(kp_eff<0 && kd_eff==0 && ki_eff==ki,"negative gains and zero terms retain their meaning");
        check(kp_eff==($signed(kp)*9)/8,"large signed baseline reaches exact 1.125 multiplier");
        kp=-3*Q;
        sample(244*Q,0);
        check(kp_eff==-27*Q/8,"runtime baseline updates without a saved gain override");

        @(negedge clk);target=300*Q;en=1;
        @(negedge clk);en=0;repeat(5)@(negedge clk);clear=1;
        repeat(40)@(negedge clk);#1;
        check(pscale==256 && dscale==256 && filtered==0,"live clear immediately resets scheduling");
        clear=0;sample(-40*Q,0);
        check(pscale==256 && dscale==256 && filtered==0,"first resumed sample ignores stale motion");
        check(dut.u_pid.integ==0 && dut.u_pid.d_lpf==0,"first resumed sample clears PID I and D history");

        rst_n=0;compare_fixed=0;compare_outer=0;repeat(4)@(negedge clk);
        kp=3*Q;ki=-Q/100;kd=24*Q;target=0;fuzzy_enable=0;rst_n=1;
        sample(0,1);compare_fixed=1;compare_outer=1;
        for(n=0;n<32;n=n+1)begin
            sample(((n*17)%31-15)*Q,0);
            check(kp_eff==kp && ki_eff==ki && kd_eff==kd,"fixed-mode telemetry reports original gains");
        end
        compare_fixed=0;fuzzy_enable=1;
        for(n=0;n<32;n=n+1)begin
            sample(((n*19)%37-18)*Q,0);
            check(outer_kp==kp && outer_ki==ki && outer_kd==kd,"outer remains fixed when fuzzy switch is enabled");
        end
        compare_outer=0;
        $display("ALL PASS conservative fuzzy PID checks=%0d",checks);$finish;
    end
    initial begin #10000000;$fatal(1,"timeout");end
endmodule
