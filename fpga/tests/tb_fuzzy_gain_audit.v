`timescale 1ns/1ps
module tb_fuzzy_gain_audit;
    localparam signed [31:0] Q=65536;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,en=0,clear=0;
    reg signed [31:0] target=0;
    wire signed [31:0] ip,ii,id,op,oi,od,out_in,fixed_out;
    wire [9:0] ps,ds;
    integer handle,read_count,vectors=0,checks=0,n,vector_clear;
    reg signed [31:0] error,exp_ip,exp_ii,exp_id,exp_op,exp_oi,exp_od;
    reg signed [31:0] fuzzy_hi,fuzzy_lo,fixed_hi,fixed_lo;
    fuzzy_pid_q16 #(.E_STEP_SHIFT(2),.EC_STEP_SHIFT(0)) inner(
        .clk(clk),.rst_n(rst_n),.en(en),.clear(clear),.fuzzy_enable(1'b1),
        .target(target),.actual(32'sd0),.kp(32'sd229376),.ki(32'sd0),.kd(100*Q),
        .out_max(1024*Q),.out_min(-1024*Q),.en_dlpf(1'b1),.en_dspan2(1'b1),
        .en_dspan4(1'b0),.lpf_shift(2'd0),.out(out_in),.kp_eff(ip),.ki_eff(ii),.kd_eff(id),
        .kp_scale_dbg(ps),.kd_scale_dbg(ds));
    fuzzy_pid_q16 #(.TUNE_ENABLE(0),.E_STEP_SHIFT(4),.EC_STEP_SHIFT(1)) outer(
        .clk(clk),.rst_n(rst_n),.en(en),.clear(clear),.fuzzy_enable(1'b1),
        .target(target),.actual(32'sd0),.kp(-32'sd98304),.ki(-32'sd1966),.kd(-32*Q),
        .out_max(750*Q),.out_min(-750*Q),.en_dlpf(1'b1),.en_dspan2(1'b1),
        .en_dspan4(1'b0),.lpf_shift(2'd0),.kp_eff(op),.ki_eff(oi),.kd_eff(od));
    pid_q16 fixed_inner(.clk(clk),.rst_n(rst_n),.en(en),.clear(clear),
        .target(target),.actual(32'sd0),.kp(32'sd229376),.ki(32'sd0),.kd(100*Q),
        .out_max(1024*Q),.out_min(-1024*Q),.en_dlpf(1'b1),.en_dspan2(1'b1),
        .en_dspan4(1'b0),.lpf_shift(2'd0),.out(fixed_out));
    task sample;input signed [31:0] e;input do_clear;begin
        @(negedge clk);target=e;clear=do_clear;en=1;
        @(negedge clk);en=0;clear=0;repeat(40)@(negedge clk);#1;
    end endtask
    task check;input ok;input [8*96-1:0] name;begin
        checks=checks+1;if(ok!==1)$fatal(1,"%0s vector=%0d e=%0d P/D=%0d/%0d gains=%0d/%0d/%0d expected=%0d/%0d/%0d",
            name,vectors,error,ps,ds,ip,ii,id,exp_ip,exp_ii,exp_id);
    end endtask
    initial begin
        repeat(4)@(negedge clk);rst_n=1;
        handle=$fopen("fuzzy_gain_vectors.txt","r");
        check(handle!=0,"generate vectors with tools/audit_fuzzy_pid.py first");
        while(!$feof(handle))begin
            read_count=$fscanf(handle,"%d %d %d %d %d %d %d %d\n",
                vector_clear,error,exp_ip,exp_ii,exp_id,exp_op,exp_oi,exp_od);
            if(read_count==8)begin
                sample(error,vector_clear!=0);vectors=vectors+1;
                check(ip===exp_ip,"stateful inner Kp matches software");
                check(ii===exp_ii,"inner Ki remains its baseline");
                check(id===exp_id,"stateful inner Kd matches software");
                check(op===exp_op,"outer Kp remains its signed baseline");
                check(oi===exp_oi,"outer Ki remains its signed baseline");
                check(od===exp_od,"outer Kd remains its signed baseline");
            end else if(read_count!=-1) $fatal(1,"malformed gain vector");
        end
        $fclose(handle);check(vectors>1000,"dense stateful vector coverage");

        sample(0,1);
        for(n=0;n<200;n=n+1)begin
            if(n%2==0)begin sample(Q,0);fuzzy_hi=out_in;fixed_hi=fixed_out;end
            else begin sample(0,0);fuzzy_lo=out_in;fixed_lo=fixed_out;end
        end
        check(inner.u_pid.d_lpf==0 && fixed_inner.d_lpf==0,"two-tap D cancels steady alternating jitter");
        check(ps==256 && ds==256,"quiet jitter cannot move scheduling multipliers");
        check(fuzzy_lo===fixed_lo && fuzzy_hi===fixed_hi,"quiet jitter amplitude equals fixed PID");
        $display("JITTER fuzzy_low_high=%0d/%0d fixed_low_high=%0d/%0d Q16",
            fuzzy_lo,fuzzy_hi,fixed_lo,fixed_hi);
        $display("ALL PASS fuzzy gain audit vectors=%0d checks=%0d",vectors,checks);$finish;
    end
    initial begin #200000000;$fatal(1,"timeout");end
endmodule
