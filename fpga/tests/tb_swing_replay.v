`timescale 1ns/1ps
module tb_swing_replay;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,req=0;
 reg [31:0] samples[0:4095];
 reg [15:0] theta=1028;
 wire [11:0] raw=theta-16'd135;
 reg signed [15:0] delta=0;
 reg signed [31:0] pos=50000;
 wire valid=raw>4 && raw<4088;
 wire balance,entered,confirmed,gap_seen,gap_active;
 wire [2:0] state;
 wire [3:0] reason;
 wire signed [15:0] drive,record_pos;
 integer i,entry=-1;
 swing_once dut(.clk(clk),.rst_n(rst),.tick(tick),.request(req),.balance_safe(1'b0),
 .sensor_valid(valid),.sensor_fresh(1'b1),.gains_valid(1'b1),.raw_angle(raw),
 .theta(theta),.center(16'd3095),.hang(16'd1017),.position(pos),.delta(delta),
 .pulse_duty(-32'sd600),.pulse_ms(32'd600),.timeout_ms(32'd1500),.travel_max(32'd450),
 .switch_deg(32'd15),.switch_dps(32'd450),.step_target(32'sd341),
 .step_kp(32'sd262144),.step_kd(32'sd104858),.capture_ms(32'd10),

 .brake_gain(32'sd0),.settle_gain(32'sd0),

 .mode_handoff(1'b0),
 .fault_code(reason),.balance(balance),.drive(drive),.state(state),.entered(entered),
 .confirmed(confirmed),.gap_seen(gap_seen),.gap_active(gap_active),.telemetry_pos(record_pos));
 task step;begin repeat(14)@(negedge clk);tick=1;@(negedge clk);tick=0;repeat(14)@(negedge clk);end endtask
 initial begin
  $readmemh("../swing_fault_replay.hex",samples);
  repeat(3)@(negedge clk);rst=1;
  for(i=1600;i<2500;i=i+1) begin
   theta=samples[i][31:16];delta=samples[i][15:0];pos=pos+delta;
   if(i==1647)req=1;
   step;
   if(state==5)$fatal(1,"replay fault idx=%0d reason=%0d",i,reason);
   if(gap_active && drive!=0)$fatal(1,"blind sector driven");
   if(balance && entry<0) begin
    entry=i;
    if(!gap_seen || record_pos==8191 || confirmed)$fatal(1,"replay diagnostics");
    $display("capture entry idx=%0d theta=%0d pos=%0d",i,theta,record_pos);
   end
  end
  if(entry<2428 || entry>2450)$fatal(1,"missed real approach %0d",entry);
  $display("ALL PASS recorded trajectory replay (not a mechanical simulation)");$finish;
 end
endmodule
