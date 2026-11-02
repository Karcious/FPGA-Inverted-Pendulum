`timescale 1ns/1ps

module tb_pid_model;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,en=0,clear=1;
    reg signed [31:0] actual=0;
    wire signed [31:0] output_raw,output_lpf,output_span2;
    integer k,handle;

    pid_q16 raw (
        .clk(clk), .rst_n(rst_n), .en(en), .clear(clear),
        .target(32'sd0), .actual(actual),
        .kp(32'sd196608), .ki(-32'sd655), .kd(32'sd1966080),
        .out_max(32'sd49152000), .out_min(-32'sd49152000),
        .en_dlpf(1'b0), .en_dspan2(1'b0),

        .en_dspan4(1'b0), .lpf_shift(2'd0),
        .out(output_raw)
    );
    pid_q16 filtered (
        .clk(clk), .rst_n(rst_n), .en(en), .clear(clear),
        .target(32'sd0), .actual(actual),
        .kp(32'sd196608), .ki(-32'sd655), .kd(32'sd1966080),
        .out_max(32'sd49152000), .out_min(-32'sd49152000),
        .en_dlpf(1'b1), .en_dspan2(1'b0),
        .en_dspan4(1'b0), .lpf_shift(2'd0),
        .out(output_lpf)
    );

    pid_q16 span2 (
        .clk(clk), .rst_n(rst_n), .en(en), .clear(clear),
        .target(32'sd0), .actual(actual),
        .kp(32'sd196608), .ki(-32'sd655), .kd(32'sd1966080),
        .out_max(32'sd49152000), .out_min(-32'sd49152000),
        .en_dlpf(1'b0), .en_dspan2(1'b1),
        .en_dspan4(1'b0), .lpf_shift(2'd0),
        .out(output_span2)
    );
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        handle=$fopen("pid_model.csv","w");
        $fdisplay(handle,"actual,clear,raw,lpf,span2");
        for(k=0;k<100;k=k+1)begin
            @(negedge clk);actual=((k*37)%200-100)*65536;clear=(k%19==0);en=1;
            @(negedge clk);en=0;repeat(10)@(negedge clk);
            $fdisplay(handle,"%0d,%0d,%0d,%0d,%0d",actual,clear,output_raw,output_lpf,output_span2);
        end
        $fclose(handle);$display("ALL PASS PID oracle generated");$finish;
    end
endmodule
