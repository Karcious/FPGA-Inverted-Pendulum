`timescale 1ns/1ps
module tb_quiet_guard;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,tick=0,eligible=1,wr_en=0;reg signed[16:0]angle=0;reg signed[15:0]delta=0;
 reg[7:0]wr_addr;reg[31:0]wr_data;wire blocked,wake;wire[127:0]readouts;integer checks=0,i;
 quiet_guard dut(clk,rst_n,tick,eligible,angle,delta,wr_en,wr_addr,wr_data,blocked,wake,readouts);
 task ct;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(2)@(negedge clk);end endtask
 task wr;input[7:0]a;input[31:0]v;begin @(negedge clk);wr_en=1;wr_addr=a;wr_data=v;@(negedge clk);wr_en=0;repeat(2)@(negedge clk);end endtask
 task check;input ok;input[511:0]msg;begin checks=checks+1;if(ok!==1)$fatal(1,"%s reason=%d still=%d",msg,dut.reason,dut.still);end endtask
 task sleep_now;begin angle=0;delta=0;eligible=1;repeat(510)ct;check(blocked,"qualified sleep");end endtask
 initial begin

  repeat(3)@(negedge clk);rst_n=1;repeat(1001)ct;
  check(blocked&&dut.enabled&&dut.wake_codes==11,"default on, 11 codes");

  wr(88,0);check(!blocked,"switch off releases");
  wr(90,6);
  wr(89,500);wr(88,1);repeat(499)ct;check(!blocked,"full duration required");ct;check(blocked,"exact duration");
  check(readouts[97:96]==3,"status bits match");
  angle=30;ct;angle=0;repeat(12)ct;check(blocked,"isolated glitch rejected");
  for(i=0;i<20;i=i+1)begin angle=(i%2);repeat(10)ct;end
  check(blocked,"alternating noise remains blocked");
  angle=2;repeat(12)ct;angle=4;repeat(12)ct;check(blocked,"below threshold");angle=6;repeat(12)ct;
  check(!blocked&&dut.reason==1,"positive sustained trend wakes");
  sleep_now;angle=-2;repeat(12)ct;angle=-4;repeat(12)ct;angle=-6;repeat(12)ct;check(!blocked&&dut.reason==1,"negative sustained trend wakes");
  sleep_now;angle=12;repeat(3)ct;check(!blocked&&dut.reason==3,"large angle bypasses trend delay");
  sleep_now;delta=1;ct;check(!blocked&&dut.reason==2,"encoder motion wakes");
  sleep_now;eligible=0;repeat(3)@(negedge clk);check(!blocked,"new task releases without tick");
  sleep_now;wr(88,0);check(!blocked&&!dut.enabled,"switch off releases immediately");
  wr(88,1);angle=8;repeat(510)ct;check(!blocked,"stationary motor but tilted is not balanced");
  angle=0;delta=0;for(i=0;i<60;i=i+1)begin angle=(i%2)?2:-2;repeat(10)ct;end
  check(!blocked,"angle span prevents false entry");
  angle=0;delta=1;repeat(510)ct;check(!blocked,"movement prevents entry");
  wr(89,1);wr(90,1);check(dut.quiet_ms==500&&dut.wake_codes==6,"invalid config rejected");
  wr(90,100);check(dut.wake_codes==100,"old upper bound removed");
  wr(90,32'hffffffff);check(readouts[95:64]==32'hffffffff,"full width threshold readback");
  delta=0;sleep_now;angle=100;repeat(30)ct;check(blocked,"double threshold does not wrap");
  $display("ALL PASS quiet guard %0d",checks);$finish;
 end
endmodule
