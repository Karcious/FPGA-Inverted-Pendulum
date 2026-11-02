`timescale 1ns/1ps
module tb_spin_trajectory;
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
 clocks(3);rst_n=1;clocks(3);
 wr(64,1);check(readouts[15:8]==1,"empty table rejected");
 wr(8'd64,32'd0);wr(65,4);wr(66,4);wr(68,1000);wr(64,1);
 check(readouts[15:8]==1&&!speed_mode,"legacy format cannot start revision 2");
 load;check(speed_mode&&velocity_q16==32768,"fractional first sample");
 ct;check(readouts[175:160]==1&&hold_actual_q16==-32768,"elapsed and positive position integral");
 pause_request=1;repeat(10)ct;check(readouts[175:160]==1&&velocity_q16==0,"pause freezes clock and requests zero");
 pause_request=0;ct;check(velocity_q16==-65536,"resume next sample signed");

 encoder=4000;ct;ct;check(speed_mode&&readouts[3]&&velocity_q16==0,"duration enters brake");
 repeat(12)ot;check(!holding,"brake waits for actual rest");
 encoder=4003;clocks(5);check(hold_actual_q16==3*65536,"brake actively opposes continued travel");
 wr(64,2);check(hold_actual_q16==3*65536,"repeated stop preserves anchor");
 encoder=4000;clocks(5);
 stopped=1;repeat(7)ot;check(!holding,"settle requires eight ticks");ot;
 check(holding&&!readouts[2]&&!speed_mode&&hold_actual_q16==0,"complete locks actual endpoint");
 encoder=4003;clocks(5);check(hold_actual_q16==3*65536,"hold feedback positive motion");
 encoder=3998;clocks(5);check(hold_actual_q16==-2*65536,"hold feedback negative motion");
 wr(64,0);wr(67,32'h00020702);check(holding&&hold_actual_q16==-2*65536,"upload reset retains endpoint");
 wr(8'd64,32'd0);wr(65,4);wr(66,4);wr(68,1000);wr(64,1);
 check(readouts[15:8]==1&&!speed_mode,"legacy format cannot start revision 2");
 load;check(speed_mode,"restart from hold");wr(64,2);check(readouts[3]&&!readouts[2],"cancel brakes");repeat(8)ot;check(holding&&!readouts[2],"cancel holds actual location");
 load;enable=0;clocks(5);check(!owned&&readouts[15:8]==3,"balance loss aborts");enable=1;clocks(5);check(!owned,"no automatic restart");
 wr(64,0);wr(67,32'h00020702);wr(65,5);wr(66,2);wr(68,0);wr(68,0);wr(64,1);check(readouts[15:8]==1,"short coverage rejected");
 wr(64,0);wr(67,32'h00020702);wr(68,167773);check(readouts[15:8]==1,"speed overflow rejected");
 load;pause_request=1;repeat(30000)ct;check(readouts[15:8]==4&&readouts[3],"pause timeout brakes");
 enable=0;clocks(5);enable=1;pause_request=0;stopped=0;
 load;wr(64,2);repeat(5000)ct;
 check(holding&&readouts[15:8]==5&&!readouts[2],"nonconvergence reported, not success");
 wr(64,0);wr(67,32'h00020702);wr(65,4000);wr(66,50);repeat(80)wr(68,65536);wr(64,1);
 for(i=0;i<3999;i=i+1) begin encoder=encoder+1;ct;end
 check(hold_actual_q16==0,"multi-turn integrated reference follows encoder without fold discontinuity");ct;
 enable=0;clocks(5);enable=1;
 wr(64,0);wr(67,32'h00020702);wr(65,4);wr(66,4);wr(68,-32768);wr(64,1);ct;
 check(hold_actual_q16==32768,"negative curve reverses reference sign");
 $display("ALL PASS spin trajectory %0d",checks);$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
