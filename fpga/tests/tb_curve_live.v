`timescale 1ns/1ps
module tb_curve_live;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,tick=0,outer_tick=0,enable=1,start_ready=1,pause_request=0,stopped=0;
 reg wr_en=0;reg[7:0]wr_addr=0;reg[31:0]wr_data=0;reg signed[31:0]encoder=0;
 wire signed[31:0] measured_velocity=65536;
 wire owned,speed_mode,holding,hold_entry,settled_entry;wire signed[31:0]velocity_q16,hold_actual_q16;wire[511:0]readouts;
 wire evade_request=0,evade_reverse=0;wire signed[31:0]evade_speed=0;spin_trajectory dut(.*);
 integer checks=0,i,old_cycles,old_elapsed,old_position,old_v;
 task check;input ok;input[511:0]msg;begin checks=checks+1;if(ok!==1)$fatal(1,"%s status=%h",msg,readouts[31:0]);end endtask
 task clocks;input integer n;begin repeat(n)@(negedge clk);end endtask
 task wr;input[7:0]a;input[31:0]v;begin @(negedge clk);wr_en=1;wr_addr=a;wr_data=v;@(negedge clk);wr_en=0;clocks(8);end endtask
 task ct;begin @(negedge clk);tick=1;@(negedge clk);tick=0;clocks(40);end endtask
 initial begin
 clocks(3);rst_n=1;clocks(5);
 wr(64,0);wr(67,32'h20702);wr(65,10);wr(66,5);wr(68,10000);wr(68,10000);wr(72,3);wr(64,1);
 repeat(10)ct;
 check(dut.cycles==1&&dut.elapsed==0&&speed_mode,"loop instead of brake at duration");
 check(hold_actual_q16<0,"loop keeps integrated reference");
 old_position=hold_actual_q16;ct;check(hold_actual_q16<old_position,"boundary does not reset position reference");
 check(velocity_q16<=242,"initial velocity ramp bounded per real millisecond");
 pause_request=1;old_elapsed=dut.elapsed;old_position=hold_actual_q16;repeat(20)ct;
 check(dut.elapsed==old_elapsed&&hold_actual_q16==old_position&&velocity_q16==0,"pause freezes phase and reference");pause_request=0;
 wr(73,32768);wr(74,131072);
 check(dut.gain==65536&&dut.rate==65536,"knobs staged atomically");wr(75,1);
 check(dut.gain==32768&&dut.rate==131072,"live commit while running");
 old_cycles=dut.cycles;repeat(10)ct;check(dut.cycles==old_cycles+2,"double rate cycles twice as fast");
 wr(73,32'hffff0000);wr(74,65536);wr(75,1);
 old_v=velocity_q16;ct;check(velocity_q16>=old_v-22&&velocity_q16<=old_v+22,"sign change is ramped");
 repeat(600)ct;check(velocity_q16==-10000,"signed live gain reaches negative target");
 check(readouts[447:416]==65536,"actual encoder speed readout");
 wr(64,2);old_cycles=dut.cycles;repeat(30)ct;
 check(readouts[3]&&velocity_q16==0&&dut.cycles==old_cycles,"stop wins over continuous loop");
 encoder=3;clocks(5);check(hold_actual_q16==3*65536,"stopping retains active restoring position loop");
 enable=0;clocks(5);enable=1;wr(64,0);wr(67,32'h20702);wr(65,10);wr(66,5);wr(68,0);wr(68,0);wr(72,2);wr(64,1);
 repeat(10)ct;check(readouts[3],"finite mode still finishes");
 $display("ALL PASS curve live %0d",checks);$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
