`timescale 1ns/1ps
module tb_adc_angle_filter;
 reg clk=0; always #10 clk=~clk;
 reg rst_n=0, sample_valid=0; reg [11:0] sample=0;
 wire [11:0] filtered; adc_angle_filter dut(.*);
 integer checks=0,errors=0,i,raw_energy=0,clean_energy=0,noise,e;
 reg [31:0] rng=32'h12345678;
 task chk(input cond,input [511:0] msg);
 begin checks=checks+1;if(!cond)begin errors=errors+1;$display("FAIL %s got=%d",msg,filtered);end end endtask
 task put(input [11:0] x);
 begin @(negedge clk);sample=x;sample_valid=1;@(negedge clk);sample_valid=0;repeat(4)@(negedge clk);end endtask
 initial begin
 repeat(3)@(negedge clk);rst_n=1;
 put(3000);chk(filtered==3000,"startup seeds actual sample");
 repeat(100)put(3000);
 put(3500);put(3000);put(3000);chk(filtered==3000,"positive impulse rejected");
 put(2500);put(3000);put(3000);chk(filtered==3000,"negative impulse rejected");
 for(i=0;i<2048;i=i+1)begin
 rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
 noise=(rng%9)-4;put(3000+noise);
 if(i>128)begin raw_energy=raw_energy+noise*noise;e=$signed({1'b0,filtered})-3000;clean_energy=clean_energy+e*e;end
 end
 $display("Noise energy raw=%0d filtered=%0d",raw_energy,clean_energy);
 chk(clean_energy*4<raw_energy,"noise RMS at least halved on test sequence");
 repeat(200)put(3000);chk(filtered==3000,"DC preserved");
 repeat(160)put(3001);chk(filtered==3001,"one-code sustained movement passes");
 repeat(200)put(3000);
 repeat(101)put(3100);chk(filtered>=3095&&filtered<=3100,"step reaches 95 percent in about 1ms");
 repeat(200)put(3000);for(i=1;i<=200;i=i+1)put(3000+i);
 chk(filtered>=3165&&filtered<=3200,"ramp lag bounded");
 repeat(300)put(4092);chk(filtered==4092,"full scale no overflow");
 put(4);put(4);chk(filtered==4,"endpoint high-low");
 put(4092);put(4092);chk(filtered==4092,"endpoint low-high");
 put(0);put(4092);put(4092);chk(filtered==4092,"isolated endpoint glitch rejected");
 @(negedge clk);rst_n=0;repeat(3)@(negedge clk);rst_n=1;
 put(1017);chk(filtered==1017,"reset seeds hanging angle");
 if(errors)$fatal(1,"%0d failed",errors);
 $display("ALL PASS: %0d checks",checks);$finish;
 end
endmodule
