module spin_trajectory(
 input wire clk,rst_n,tick,outer_tick,enable,start_ready,pause_request,stopped,
 input wire wr_en,input wire [7:0] wr_addr,input wire [31:0] wr_data,
 input wire evade_request,evade_reverse,
 input wire signed [31:0] evade_speed,
 input wire signed [31:0] encoder,
 input wire signed [31:0] measured_velocity,
 output wire owned,speed_mode,holding,
 output wire signed [31:0] velocity_q16,hold_actual_q16,
 output reg settled_entry,
 output reg hold_entry,output wire [511:0] readouts
);
 localparam IDLE=0,RUN=1,BRAKE=2,HOLD=3,PREP=4;
 reg [2:0] state;reg start_armed;reg handoff_on_settle;reg [15:0] prepare_age;reg [1:0] warm;
 reg signed [31:0] table_data[0:1023];reg signed [31:0] sample;
 reg [31:0] duration,interval,checksum,last_data,coverage;
 reg [10:0] loaded;reg [9:0] index;
 reg [15:0] elapsed,within_sample,pause_age;reg [3:0] settle;
 reg [7:0] fault;reg completed,config_ok,format_ok;
 reg signed [31:0] encoder_previous;
 reg signed [31:0] follow_error;
 reg [12:0] brake_age;
 reg [31:0] options,cycles;
 reg signed [31:0] gain_pending,gain;
 reg [31:0] rate_pending,rate;
 reg [19:0] phase_budget;
 reg phase_gap;
 reg signed [63:0] scaled_product;
 reg signed [31:0] scaled_target,command_velocity;
 reg evade_session;
 wire live_mode=options[1] || evade_session;
 wire signed [31:0] ramp_target=evade_session ? (evade_request ? (evade_reverse ? evade_speed : -evade_speed) : 32'sd0) : scaled_target;
 wire virtual_tick=live_mode ? (phase_budget>=65536 && !phase_gap) : tick;
 wire signed [31:0] requested_velocity=live_mode ? command_velocity : sample;
 always @(posedge clk) begin
  scaled_product<=sample*gain;
  if((scaled_product>>>16)>167772) scaled_target<=167772;
  else if((scaled_product>>>16)<-167772) scaled_target<=-167772;
  else scaled_target<=scaled_product>>>16;
 end

 wire busy=state==RUN || state==BRAKE;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   options<=0;cycles<=0;gain<=65536;gain_pending<=65536;rate<=65536;rate_pending<=65536;
   phase_budget<=0;phase_gap<=0;command_velocity<=0;
  end else begin
   phase_gap<=0;
   if(state!=RUN || pause_request) phase_budget<=0;
   else if(tick) phase_budget<={4'd0,phase_budget[15:0]}+rate;
   else if(virtual_tick && live_mode) begin phase_budget<=phase_budget-65536;phase_gap<=1;end
   if(state!=RUN || pause_request) command_velocity<=0;
   else if(tick) begin
    if(ramp_target>command_velocity+22) command_velocity<=command_velocity+22;
    else if(ramp_target<command_velocity-22) command_velocity<=command_velocity-22;
    else command_velocity<=ramp_target;
   end
   if(wr_en) begin
    if(wr_addr==64 && wr_data==0 && !busy) begin
     options<=0;cycles<=0;gain<=65536;gain_pending<=65536;rate<=65536;rate_pending<=65536;
    end
    if(wr_addr==72 && !busy) options<=wr_data & 3;
    if(wr_addr==73 && $signed(wr_data)>=-524288 && $signed(wr_data)<=524288) gain_pending<=wr_data;
    if(wr_addr==74 && wr_data>=8192 && wr_data<=262144) rate_pending<=wr_data;

    if(wr_addr==75 && wr_data==1 && state!=BRAKE) begin gain<=gain_pending;rate<=rate_pending;end
   end
   if(!evade_session && state==RUN && virtual_tick && !pause_request && elapsed+1>=duration && options[0]) cycles<=cycles+1'b1;
  end
 end

 wire signed [15:0] encoder_backstep=encoder_previous-encoder;
 wire signed [32:0] advance=(tick && state==RUN && (&warm) && !pause_request) ?
                            {requested_velocity[31],requested_velocity} : 33'sd0;
 wire signed [32:0] error_next=$signed({follow_error[31],follow_error})+
             $signed({encoder_backstep[15],encoder_backstep,16'd0})+advance;

 assign owned=state!=IDLE && !(state==HOLD && handoff_on_settle && fault==0);
 assign speed_mode=busy && enable;
 assign holding=state==HOLD && enable;
 assign velocity_q16=(state==RUN && (&warm) && !pause_request) ? requested_velocity : 32'sd0;
 assign hold_actual_q16=-follow_error;
 assign readouts={32'd0,{30'd0,enable,start_ready},measured_velocity,cycles,32'd0,rate,gain,options,velocity_q16,{21'd0,loaded},{16'd0,elapsed},last_data,checksum,interval,duration,
  16'd0,fault,1'b0,start_armed,(state==PREP),holding,(state==BRAKE),(holding && completed),(state==RUN && pause_request),(busy || state==PREP)};
 always @(posedge clk) begin
  sample<=table_data[index];coverage<=loaded*interval;
  config_ok<=format_ok && duration>=1 && duration<=60000 && interval>=1 && interval<=60 && loaded>=1 &&
             coverage>=duration && coverage-interval<duration;
  if(wr_en && wr_addr==68 && !busy && loaded<1024 && fault==0 &&
     $signed(wr_data)>=-167772 && $signed(wr_data)<=167772)
   table_data[loaded[9:0]]<=wr_data;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin encoder_previous<=0;follow_error<=0;end
  else begin
   encoder_previous<=encoder;
   if(hold_entry || !owned) follow_error<=0;
   else if(error_next>34078720) follow_error<=34078720;
   else if(error_next< -34078720) follow_error<=-34078720;
   else follow_error<=error_next[31:0];
  end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   evade_session<=0;state<=IDLE;warm<=0;duration<=0;interval<=0;checksum<=0;last_data<=0;loaded<=0;index<=0;
   elapsed<=0;within_sample<=0;pause_age<=0;settle<=0;fault<=0;completed<=0;hold_entry<=0;brake_age<=0;format_ok<=0;start_armed<=0;prepare_age<=0;settled_entry<=0;handoff_on_settle<=0;
  end else begin
   hold_entry<=0;settled_entry<=0;
   if(state!=RUN) warm<=0;else warm<={warm[0],1'b1};

   if(state!=IDLE && !enable) begin evade_session<=0;state<=IDLE;if(!evade_session)fault<=3;completed<=0;start_armed<=0;handoff_on_settle<=0;end
   else if(evade_request && enable && start_ready && !owned && !(wr_en && wr_addr==64)) begin
    evade_session<=1;state<=RUN;warm<=0;hold_entry<=1;handoff_on_settle<=1;fault<=0;completed<=0;pause_age<=0;
   end else if(wr_en && wr_addr==64 && wr_data==3 && !busy) begin
    if(enable) begin evade_session<=0;state<=PREP;handoff_on_settle<=1;hold_entry<=1;start_armed<=0;prepare_age<=0;fault<=0;completed<=0;end
    else fault<=1;
   end else if(wr_en && wr_addr==64 && wr_data==2) begin
    start_armed<=0;
    if(state==PREP) begin state<=BRAKE;settle<=0;brake_age<=0;end
    if(state==RUN) begin state<=BRAKE;settle<=0;brake_age<=0;hold_entry<=1;end
    completed<=0;
   end else if(wr_en && wr_addr==64 && wr_data==0 && !busy) begin

    evade_session<=0;loaded<=0;checksum<=0;last_data<=0;index<=0;elapsed<=0;within_sample<=0;format_ok<=0;
    pause_age<=0;duration<=0;interval<=0;fault<=0;completed<=0;start_armed<=0;prepare_age<=0;
   end else if(state==PREP && start_armed && tick) begin
    if(start_ready && config_ok && enable) begin
     state<=RUN;start_armed<=0;warm<=0;index<=0;elapsed<=0;within_sample<=0;pause_age<=0;completed<=0;hold_entry<=1;
    end else if(prepare_age>=29999) begin state<=BRAKE;start_armed<=0;fault<=6;settle<=0;brake_age<=0;end
    else prepare_age<=prepare_age+1'b1;
   end else if(state==RUN && evade_session && tick && !evade_request && command_velocity==0) begin

    state<=HOLD;settle<=0;brake_age<=0;hold_entry<=1;settled_entry<=1;completed<=1;
   end else if(state==RUN && (virtual_tick || (pause_request && tick)) && (&warm)) begin
    if(pause_request) begin
     if(pause_age>=29999) begin state<=BRAKE;if(!evade_session)fault<=4;completed<=0;settle<=0;brake_age<=0;hold_entry<=1;end
     else pause_age<=pause_age+1'b1;
    end else begin
     pause_age<=0;
     if(!evade_session)begin
     elapsed<=elapsed+1'b1;
     if(elapsed+1>=duration) begin
      if(options[0]) begin elapsed<=0;index<=0;within_sample<=0;end
      else begin state<=BRAKE;completed<=1;settle<=0;brake_age<=0;hold_entry<=1;end
     end
     else if(within_sample+1>=interval) begin index<=index+1'b1;within_sample<=0;end
     else within_sample<=within_sample+1'b1;
    end
   end
   end else if(state==BRAKE && tick) begin

    if(brake_age>=4999) begin
     state<=HOLD;
     if(evade_session) begin settled_entry<=1;hold_entry<=1;end
     else fault<=5;
     completed<=0;
    end
    else brake_age<=brake_age+1'b1;
   end else if(state==BRAKE && outer_tick) begin
    if(stopped) begin
     if(settle==7) begin state<=HOLD;settle<=0;settled_entry<=1;hold_entry<=1;end
     else settle<=settle+1'b1;
    end else settle<=0;
   end else if(wr_en && !busy) begin
    if(wr_addr==64 && wr_data==1) begin
     if(config_ok && start_ready && enable && fault==0) begin
      state<=RUN;warm<=0;index<=0;elapsed<=0;within_sample<=0;pause_age<=0;completed<=0;hold_entry<=1;
     end else if(config_ok && enable && fault==0 && state==PREP) begin start_armed<=1;prepare_age<=0;end
     else fault<=1;
    end else if(wr_addr==65) duration<=wr_data;
    else if(wr_addr==66) interval<=wr_data;
    else if(wr_addr==67) format_ok<=wr_data==32'h00020702;
    else if(wr_addr==68) begin
     if(loaded>=1024 || $signed(wr_data)<-167772 || $signed(wr_data)>167772) fault<=1;
     else if(fault==0) begin loaded<=loaded+1'b1;checksum<=checksum+wr_data;last_data<=wr_data;end
    end
   end
  end
 end
endmodule
