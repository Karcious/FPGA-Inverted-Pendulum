`timescale 1ns/1ps
module tb_curve_takeover;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,tick=0,outer_tick=0,enable=1,start_ready=1,pause_request=0,stopped=0;
 reg wr_en=0;reg[7:0]wr_addr=0;reg[31:0]wr_data=0;reg signed[31:0]encoder=0;
 wire owned,speed_mode,holding,hold_entry,settled_entry;wire signed[31:0]velocity_q16,hold_actual_q16;wire[511:0]readouts;
 wire evade_request=0,evade_reverse=0;wire signed[31:0]evade_speed=0;spin_trajectory dut(.*);
 wire signed [31:0] measured_velocity=0;
 integer checks=0,i;
 task check;input ok;input[511:0]msg;begin checks=checks+1;if(ok!==1)$fatal(1,"%s status=%h",msg,readouts[31:0]);end endtask
 task clocks;input integer n;begin repeat(n)@(negedge clk);end endtask
 task wr;input[7:0]a;input[31:0]v;begin @(negedge clk);wr_en=1;wr_addr=a;wr_data=v;@(negedge clk);wr_en=0;clocks(4);end endtask
 task ct;begin @(negedge clk);tick=1;@(negedge clk);tick=0;clocks(3);end endtask
 task ot;begin @(negedge clk);outer_tick=1;@(negedge clk);outer_tick=0;clocks(3);end endtask
 task load;begin wr(64,0);wr(67,32'h00020702);wr(65,4);wr(66,2);wr(68,32768);wr(68,-65536);wr(64,1);end endtask
 initial begin
 clocks(3);rst_n=1;clocks(3);start_ready=0;
 wr(64,3);check(owned&&readouts[5]&&readouts[0]&&velocity_q16==0,"claim holds and reports preparation");
 encoder=3;clocks(5);check(hold_actual_q16==3*65536,"claim actively parks moving arm");
 wr(64,0);check(owned&&readouts[5]&&hold_actual_q16==3*65536,"upload reset keeps claim and anchor");
 wr(67,32'h20702);wr(65,10);wr(66,10);wr(68,1000);wr(64,1);
 check(readouts[5]&&readouts[6]&&readouts[15:8]==0,"unready start queues without fault");
 repeat(20)ct;check(readouts[175:160]==0&&velocity_q16==0,"waiting does not consume trajectory time");
 start_ready=1;ct;clocks(5);check(!readouts[5]&&speed_mode,"ready automatically starts uploaded curve");
 repeat(10)ct;check(readouts[3],"finite curve parks");
 stopped=1;encoder=4;clocks(5);repeat(8)ot;
 check(holding&&hold_actual_q16==0,"settled hold reanchors at actual endpoint");
 wr(64,3);start_ready=0;wr(64,0);wr(67,32'h20702);wr(65,10);wr(66,10);wr(68,0);wr(64,1);
 wr(64,2);start_ready=1;repeat(20)ct;check(readouts[3]&&!readouts[6],"stop cancels pending start");
 repeat(8)ot;wr(64,3);start_ready=0;wr(64,0);wr(67,32'h20702);wr(65,10);wr(66,10);wr(68,0);wr(64,1);
 repeat(30000)ct;check(readouts[15:8]==6&&readouts[3],"preparation timeout parks without launching");
 enable=0;clocks(5);check(!owned,"global stop releases claim");
 $display("ALL PASS curve takeover %0d",checks);$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
