module quiet_guard(input wire clk,rst_n,tick,eligible,
 input wire signed [16:0] angle,input wire signed [15:0] delta,
 input wire wr_en,input wire [7:0] wr_addr,input wire [31:0] wr_data,
 output reg blocked,output reg wake,output wire [127:0] readouts);
 reg enabled;reg [15:0] quiet_ms;reg [31:0] wake_codes;
 reg [15:0] still;reg [7:0] reason;
 reg signed [16:0] h1,h2,lo,hi,previous;
 reg [3:0] divider;reg [1:0] trend;reg [6:0] trend_age;
 wire signed [16:0] median=(angle>h1)?((h1>h2)?h1:((angle>h2)?h2:angle)):((angle>h2)?angle:((h1>h2)?h2:h1));
 wire [16:0] mag=median[16] ? -median : median;
 wire [16:0] prev_mag=previous[16] ? -previous : previous;
 wire signed [17:0] span=$signed(hi)-$signed(lo);
 assign readouts={{15'd0,reason,7'd0,blocked,enabled},wake_codes,{16'd0,quiet_ms},{31'd0,enabled}};

 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin

   enabled<=1;quiet_ms<=1000;wake_codes<=11;blocked<=0;wake<=0;still<=0;reason<=0;
   h1<=0;h2<=0;lo<=0;hi<=0;previous<=0;divider<=0;trend<=0;trend_age<=0;
  end else begin
   wake<=0;
   if(wr_en && wr_addr==89 && wr_data>=500 && wr_data<=5000 && !blocked) quiet_ms<=wr_data[15:0];
   if(wr_en && wr_addr==90 && wr_data>=4 && !blocked) wake_codes<=wr_data;
   if(wr_en && wr_addr==88 && wr_data<=1) begin
    enabled<=wr_data[0];blocked<=0;wake<=blocked;still<=0;trend<=0;reason<=5;
   end else if(!eligible || !enabled) begin
    blocked<=0;wake<=blocked;still<=0;trend<=0;divider<=0;trend_age<=0;
    if(blocked)reason<=4;
   end else if(tick) begin
    h2<=h1;h1<=angle;
    if(!blocked) begin
     trend<=0;divider<=0;trend_age<=0;previous<=median;
     if(delta!=0 || mag>3) begin still<=0;lo<=median;hi<=median;end
     else if(still==0 || span>2 || median>lo+2 || median<hi-2) begin still<=1;lo<=median;hi<=median;end
     else begin
      if(median<lo)lo<=median;if(median>hi)hi<=median;
      if(still>=quiet_ms-1) begin blocked<=1;reason<=0;still<=0;end
      else still<=still+1'b1;
     end
    end else begin
     if(delta!=0 || mag>={wake_codes,1'b0}) begin
      blocked<=0;wake<=1;still<=0;trend<=0;reason<=(delta!=0)?2:3;
     end else if(divider==9) begin
      divider<=0;previous<=median;

      if(median[16]!=previous[16] || mag<prev_mag || mag==0) begin trend<=0;trend_age<=0;end
      else if(mag>prev_mag) begin
       trend_age<=0;if(trend!=3)trend<=trend+1'b1;
       if(mag>=wake_codes && trend>=1) begin blocked<=0;wake<=1;still<=0;reason<=1;end
      end else if(trend_age>=9) begin trend<=0;trend_age<=0;end
      else trend_age<=trend_age+1'b1;
     end else divider<=divider+1'b1;
    end
   end
  end
 end
endmodule
