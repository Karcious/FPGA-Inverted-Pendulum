module tof_receiver(input clk,input rst_n,input tick,input wr_en,
 input [7:0] wr_addr,input [31:0] wr_data,output [255:0] readouts, input permit, output evade_request, output reverse, output reg signed [31:0] speed);
 reg [31:0] left,right,seq,stage_left,stage_right;
 reg [1:0] flags,stage_flags;
 reg [9:0] age;
 reg [2:0] staged;
 reg armed,away,reverse_r;reg [7:0] clear_ms;
 wire fresh=age<500;
 assign evade_request=armed && away && fresh && flags[0] && permit;
 assign reverse=reverse_r;

 reg [7:0] speed_deg;

 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin speed_deg<=15;speed<=2955;end
  else begin
   speed_deg<=left>=90 ? 8'd15 : left<=15 ? 8'd90 : 8'd105-left[7:0];
   speed<=speed_deg*32'sd197;
  end
 end
 assign readouts={{29'd0,evade_request,flags[0]&&fresh,armed},{30'd0,reverse_r,armed},{22'd0,age},{29'd0,fresh,(fresh ? flags : 2'b0)},seq,right,left,32'h544f4631};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin armed<=0;away<=0;reverse_r<=0;clear_ms<=0;end
  else begin
   if(!permit)begin armed<=0;away<=0;clear_ms<=0;end
   else if(wr_en && wr_addr==86)begin armed<=wr_data[0];reverse_r<=wr_data[1];away<=0;clear_ms<=0;end
   else if(tick)begin
    if(!armed || !fresh || !flags[0])begin away<=0;clear_ms<=0;end
    else if(left<60)begin away<=1;clear_ms<=0;end
    else if(left>90)begin
     if(clear_ms>=199)away<=0;else clear_ms<=clear_ms+1'b1;
    end else clear_ms<=0;
   end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin left<=0;right<=0;seq<=0;stage_left<=0;stage_right<=0;flags<=0;stage_flags<=0;age<=500;staged<=0;end
  else begin
   if(tick && age<500)age<=age+1'b1;
   if(wr_en)case(wr_addr)
    81:begin stage_left<=wr_data;staged[0]<=1;end
    82:begin stage_right<=wr_data;staged[1]<=1;end
    84:begin stage_flags<=wr_data[1:0];staged[2]<=1;end
    83:begin
     if(staged==7)begin left<=stage_left;right<=stage_right;flags<=stage_flags;seq<=wr_data;age<=0;end
     staged<=0;
    end
   endcase
  end
 end
endmodule
