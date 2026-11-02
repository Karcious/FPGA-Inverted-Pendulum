`timescale 1ns/1ps
module tb_avoid;
reg clk=0;always #5 clk=~clk;
reg rst_n=0,tick=0,outer_tick=0,enable=1,start_ready=1,pause_request=0,stopped=1;
reg wr_en=0;reg[7:0]wr_addr=0;reg[31:0]wr_data=0;
wire signed[31:0]speed;wire req,rev,owned,speed_mode,holding,settled_entry,hold_entry;
wire[255:0]tof;wire[511:0]readouts;
wire signed[31:0]velocity_q16,hold_actual_q16;
tof_receiver t(clk,rst_n,tick,wr_en,wr_addr,wr_data,tof,enable,req,rev,speed);
spin_trajectory c(.clk(clk),.rst_n(rst_n),.tick(tick),.outer_tick(outer_tick),.enable(enable),.start_ready(start_ready),.pause_request(pause_request),.stopped(stopped),.wr_en(wr_en),.wr_addr(wr_addr),.wr_data(wr_data),.evade_request(req),.evade_reverse(rev),.evade_speed(speed),.encoder(32'sd0),.measured_velocity(32'sd0),.owned(owned),.speed_mode(speed_mode),.holding(holding),.settled_entry(settled_entry),.hold_entry(hold_entry),.velocity_q16(velocity_q16),.hold_actual_q16(hold_actual_q16),.readouts(readouts));
task wr(input[7:0]a,input[31:0]v);begin @(negedge clk);wr_en=1;wr_addr=a;wr_data=v;@(negedge clk);wr_en=0;repeat(4)@(negedge clk);end endtask
task packet(input[31:0]distance,input[31:0]flags);begin wr(81,distance);wr(82,0);wr(84,flags);wr(83,1);end endtask
task ms;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(4)@(negedge clk);outer_tick=1;@(negedge clk);outer_tick=0;repeat(4)@(negedge clk);end endtask
initial begin
repeat(3)@(negedge clk);rst_n=1;packet(40,1);repeat(10)ms;
if(owned)$fatal(1,"disabled moved");
wr(86,1);repeat(10)begin packet(40,1);repeat(100)ms;end
if(!owned || velocity_q16!=-12805)$fatal(1,"near does not retreat/ramp");
packet(20,1);repeat(4)begin packet(20,1);repeat(100)ms;end
if(velocity_q16!=-16745)$fatal(1,"closer obstacle must increase speed");
packet(75,1);repeat(7)begin packet(75,1);repeat(100)ms;end
if(velocity_q16!=-5910)$fatal(1,"farther obstacle must reduce speed");
packet(75,1);repeat(20)ms;if(!req)$fatal(1,"hysteresis lost");
packet(100,1);repeat(190)ms;if(!req)$fatal(1,"cleared early");
repeat(20)ms;if(req)$fatal(1,"did not clear");
repeat(700)ms;if(owned || velocity_q16!=0)$fatal(1,"did not park");
packet(40,1);repeat(10)begin packet(40,1);repeat(100)ms;end if(!owned)$fatal(1,"did not retrigger");
repeat(1200)ms;if(req || owned)$fatal(1,"stale did not park");
wr(86,3);packet(40,1);repeat(10)begin packet(40,1);repeat(100)ms;end if(velocity_q16!=12805)$fatal(1,"reverse direction");
packet(40,0);repeat(700)ms;if(owned)$fatal(1,"invalid data did not park");
packet(40,1);repeat(10)begin packet(40,1);repeat(100)ms;end pause_request=1;repeat(3)ms;if(velocity_q16!=0)$fatal(1,"balance pause ignored");pause_request=0;

if(readouts[15:8]!=0)$fatal(1,"normal evasion latched a fault");

stopped=0;packet(40,1);repeat(10)begin packet(40,1);repeat(100)ms;end
if(!owned)$fatal(1,"stopped=0 case did not start");
packet(200,1);

repeat(1200)ms;
if(readouts[15:8]!=0)$fatal(1,"evade exit latched a fault");
if(owned)$fatal(1,"evade exit kept ownership");
if(!readouts[4])$fatal(1,"evade exit did not reach HOLD");
stopped=1;
enable=0;repeat(3)ms;if(owned || req || tof[192])$fatal(1,"stop did not disarm");
$display("ALL PASS single-side avoidance: ramp hysteresis stop stale repeat reverse invalid balance stop no-fault timeout-release");$finish;
end
endmodule
