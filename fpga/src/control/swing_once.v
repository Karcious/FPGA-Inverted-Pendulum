`timescale 1ns/1ps
module swing_once(
    input wire clk, rst_n, tick, request, balance_safe, sensor_valid, gains_valid,
    input wire [15:0] theta, center, hang,
    input wire [11:0] raw_angle,
    input wire sensor_fresh,
    input wire signed [31:0] position,
    input wire signed [15:0] delta,
    input wire signed [31:0] pulse_duty,
    input wire [31:0] pulse_ms, timeout_ms, travel_max,
    input wire [31:0] switch_deg, switch_dps,
    input wire signed [31:0] step_target, step_kp, step_kd,
    input wire [31:0] capture_ms,

    input wire signed [31:0] brake_gain,

    input wire signed [31:0] settle_gain,

    input wire mode_handoff,
    output reg [3:0] fault_code,
    output reg entered, confirmed, gap_seen, gap_active,
    output wire signed [15:0] telemetry_pos,
    output wire balance,
    output reg signed [15:0] drive,
    output reg [2:0] state
);
    localparam IDLE=0, REST=1, POSITION=2, WAIT_UP=3, BALANCE=4, FAULT=5, CAPTURE=6;

    localparam [1:0]  RETRY_MAX  = 2'd3;
    localparam [7:0]  RETRY_STEP = 8'd20;
    localparam signed [31:0] SWING_TARGET_MAX = 32'sd400;
    reg  [1:0] retry_count;
    wire retryable = (fault_code==4'd3) ||
                     (fault_code==4'd5) ||
                     (fault_code==4'd7) ||
                     (fault_code==4'd8) || (fault_code==4'd9) ||
                     (fault_code==4'd10);

    localparam [2:0]  REST_MAX   = 3'd6;
    reg  [2:0] rest_count;

    reg  [1:0] swing_idx;

    wire [7:0] retry_bump_u = {6'd0, swing_idx} * RETRY_STEP;
    wire signed [31:0] swing_raw = step_target + $signed({24'd0, retry_bump_u});

    wire signed [31:0] swing_target = swing_raw > SWING_TARGET_MAX ?
                                      SWING_TARGET_MAX : swing_raw;
    reg [5:0] gap_age;
    reg [9:0] capture_age;
    reg [11:0] previous_raw;
    wire pre_capture = state==POSITION || state==WAIT_UP;
    wire near_edge = raw_angle<64 || raw_angle>4028;
    wire edge_wrap = (previous_raw<256 && raw_angle>3836) ||
                     (previous_raw>3836 && raw_angle<256);
    wire edge_approach = previous_raw<128 || previous_raw>3964;

    wire gap_allowed = pre_capture && sensor_fresh &&
                       (gap_active || (near_edge && edge_approach) || edge_wrap);

    localparam [15:0] RAW_STEP_MAX = 16'd160;
    wire [15:0] raw_step = raw_angle>previous_raw ? raw_angle-previous_raw
                                                  : previous_raw-raw_angle;
    wire raw_trusted = (raw_step<=RAW_STEP_MAX) || edge_wrap || !sensor_valid;
    reg [15:0] elapsed;
    reg [7:0] stable;
    reg signed [31:0] origin;
    reg [15:0] history [0:19];
    reg [4:0] history_idx;
    reg [5:0] valid_count;
    reg [15:0] previous;
    integer k;
    wire signed [16:0] velocity = $signed({1'b0,theta}) - $signed({1'b0,history[history_idx]});
    wire [16:0] velocity_abs = velocity[16] ? -velocity : velocity;
    wire signed [32:0] travel = {position[31],position}-{origin[31],origin};
    wire [15:0] dp_abs = delta[15] ? -delta : delta;
    wire [15:0] angle_abs = theta>center ? theta-center : center-theta;
    wire [15:0] hang_abs = theta>hang ? theta-hang : hang-theta;
    wire [15:0] jump = theta>previous ? theta-previous : previous-theta;

    localparam [2:0] JUMP_DEBOUNCE = 3'd4;
    reg  [2:0] jump_bad;
    wire jump_glitch = state!=REST && state!=FAULT && !gap_allowed &&
                       valid_count!=0 && jump>1024;
    wire jump_dead = (jump_bad==JUMP_DEBOUNCE-1'b1);

    reg [15:0] capture_angle, capture_velocity;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin capture_angle<=0; capture_velocity<=0; end
        else begin
            case(switch_deg)
                32'd5: capture_angle<=16'd57;
                32'd6: capture_angle<=16'd69;
                32'd7: capture_angle<=16'd80;
                32'd8: capture_angle<=16'd92;
                32'd9: capture_angle<=16'd103;
                32'd10: capture_angle<=16'd115;
                32'd11: capture_angle<=16'd126;
                32'd12: capture_angle<=16'd138;
                32'd13: capture_angle<=16'd150;
                32'd14: capture_angle<=16'd161;
                32'd15: capture_angle<=16'd173;
                32'd16: capture_angle<=16'd184;
                32'd17: capture_angle<=16'd196;
                32'd18: capture_angle<=16'd207;
                32'd19: capture_angle<=16'd219;
                32'd20: capture_angle<=16'd230;
                32'd21: capture_angle<=16'd242;
                32'd22: capture_angle<=16'd253;
                32'd23: capture_angle<=16'd265;
                32'd24: capture_angle<=16'd277;
                32'd25: capture_angle<=16'd288;
                32'd26: capture_angle<=16'd300;
                32'd27: capture_angle<=16'd311;
                32'd28: capture_angle<=16'd323;
                32'd29: capture_angle<=16'd334;
                32'd30: capture_angle<=16'd346;
                default: capture_angle<=0;
            endcase
            case(switch_dps)
                32'd10, 32'd11, 32'd12: capture_velocity<=16'd2;
                32'd13, 32'd14, 32'd15, 32'd16, 32'd17: capture_velocity<=16'd3;
                32'd18, 32'd19, 32'd20, 32'd21: capture_velocity<=16'd4;
                32'd22, 32'd23, 32'd24, 32'd25: capture_velocity<=16'd5;
                32'd26, 32'd27, 32'd28, 32'd29, 32'd30: capture_velocity<=16'd6;
                32'd31, 32'd32, 32'd33, 32'd34: capture_velocity<=16'd7;
                32'd35, 32'd36, 32'd37, 32'd38: capture_velocity<=16'd8;
                32'd39, 32'd40, 32'd41, 32'd42, 32'd43: capture_velocity<=16'd9;
                32'd44, 32'd45, 32'd46, 32'd47: capture_velocity<=16'd10;
                32'd48, 32'd49, 32'd50, 32'd51: capture_velocity<=16'd11;
                32'd52, 32'd53, 32'd54, 32'd55, 32'd56: capture_velocity<=16'd12;
                32'd57, 32'd58, 32'd59, 32'd60: capture_velocity<=16'd13;
                32'd61, 32'd62, 32'd63, 32'd64: capture_velocity<=16'd14;
                32'd65, 32'd66, 32'd67, 32'd68, 32'd69: capture_velocity<=16'd15;
                32'd70, 32'd71, 32'd72, 32'd73: capture_velocity<=16'd16;
                32'd74, 32'd75, 32'd76, 32'd77: capture_velocity<=16'd17;
                32'd78, 32'd79, 32'd80, 32'd81, 32'd82: capture_velocity<=16'd18;
                32'd83, 32'd84, 32'd85, 32'd86: capture_velocity<=16'd19;
                32'd87, 32'd88, 32'd89, 32'd90: capture_velocity<=16'd20;
                32'd91, 32'd92, 32'd93, 32'd94, 32'd95: capture_velocity<=16'd21;
                32'd96, 32'd97, 32'd98, 32'd99: capture_velocity<=16'd22;
                32'd100, 32'd101, 32'd102, 32'd103: capture_velocity<=16'd23;
                32'd104, 32'd105, 32'd106, 32'd107, 32'd108: capture_velocity<=16'd24;
                32'd109, 32'd110, 32'd111, 32'd112: capture_velocity<=16'd25;
                32'd113, 32'd114, 32'd115, 32'd116: capture_velocity<=16'd26;
                32'd117, 32'd118, 32'd119, 32'd120, 32'd121: capture_velocity<=16'd27;
                32'd122, 32'd123, 32'd124, 32'd125: capture_velocity<=16'd28;
                32'd126, 32'd127, 32'd128, 32'd129: capture_velocity<=16'd29;
                32'd130, 32'd131, 32'd132, 32'd133, 32'd134: capture_velocity<=16'd30;
                32'd135, 32'd136, 32'd137, 32'd138: capture_velocity<=16'd31;
                32'd139, 32'd140, 32'd141, 32'd142: capture_velocity<=16'd32;
                32'd143, 32'd144, 32'd145, 32'd146, 32'd147: capture_velocity<=16'd33;
                32'd148, 32'd149, 32'd150, 32'd151: capture_velocity<=16'd34;
                32'd152, 32'd153, 32'd154, 32'd155: capture_velocity<=16'd35;
                32'd156, 32'd157, 32'd158, 32'd159, 32'd160: capture_velocity<=16'd36;
                32'd161, 32'd162, 32'd163, 32'd164: capture_velocity<=16'd37;
                32'd165, 32'd166, 32'd167, 32'd168: capture_velocity<=16'd38;
                32'd169, 32'd170, 32'd171, 32'd172, 32'd173: capture_velocity<=16'd39;
                32'd174, 32'd175, 32'd176, 32'd177: capture_velocity<=16'd40;
                32'd178, 32'd179, 32'd180, 32'd181: capture_velocity<=16'd41;
                32'd182, 32'd183, 32'd184, 32'd185, 32'd186: capture_velocity<=16'd42;
                32'd187, 32'd188, 32'd189, 32'd190: capture_velocity<=16'd43;
                32'd191, 32'd192, 32'd193, 32'd194: capture_velocity<=16'd44;
                32'd195, 32'd196, 32'd197, 32'd198, 32'd199: capture_velocity<=16'd45;
                32'd200, 32'd201, 32'd202, 32'd203: capture_velocity<=16'd46;
                32'd204, 32'd205, 32'd206, 32'd207: capture_velocity<=16'd47;
                32'd208, 32'd209, 32'd210, 32'd211, 32'd212: capture_velocity<=16'd48;
                32'd213, 32'd214, 32'd215, 32'd216: capture_velocity<=16'd49;
                32'd217, 32'd218, 32'd219, 32'd220: capture_velocity<=16'd50;
                32'd221, 32'd222, 32'd223, 32'd224, 32'd225: capture_velocity<=16'd51;
                32'd226, 32'd227, 32'd228, 32'd229: capture_velocity<=16'd52;
                32'd230, 32'd231, 32'd232, 32'd233: capture_velocity<=16'd53;
                32'd234, 32'd235, 32'd236, 32'd237, 32'd238: capture_velocity<=16'd54;
                32'd239, 32'd240, 32'd241, 32'd242: capture_velocity<=16'd55;
                32'd243, 32'd244, 32'd245, 32'd246: capture_velocity<=16'd56;
                32'd247, 32'd248, 32'd249, 32'd250, 32'd251: capture_velocity<=16'd57;
                32'd252, 32'd253, 32'd254, 32'd255: capture_velocity<=16'd58;
                32'd256, 32'd257, 32'd258, 32'd259: capture_velocity<=16'd59;
                32'd260, 32'd261, 32'd262, 32'd263, 32'd264: capture_velocity<=16'd60;
                32'd265, 32'd266, 32'd267, 32'd268: capture_velocity<=16'd61;
                32'd269, 32'd270, 32'd271, 32'd272: capture_velocity<=16'd62;
                32'd273, 32'd274, 32'd275, 32'd276, 32'd277: capture_velocity<=16'd63;
                32'd278, 32'd279, 32'd280, 32'd281: capture_velocity<=16'd64;
                32'd282, 32'd283, 32'd284, 32'd285: capture_velocity<=16'd65;
                32'd286, 32'd287, 32'd288, 32'd289, 32'd290: capture_velocity<=16'd66;
                32'd291, 32'd292, 32'd293, 32'd294: capture_velocity<=16'd67;
                32'd295, 32'd296, 32'd297, 32'd298: capture_velocity<=16'd68;
                32'd299, 32'd300, 32'd301, 32'd302, 32'd303: capture_velocity<=16'd69;
                32'd304, 32'd305, 32'd306, 32'd307: capture_velocity<=16'd70;
                32'd308, 32'd309, 32'd310, 32'd311: capture_velocity<=16'd71;
                32'd312, 32'd313, 32'd314, 32'd315, 32'd316: capture_velocity<=16'd72;
                32'd317, 32'd318, 32'd319, 32'd320: capture_velocity<=16'd73;
                32'd321, 32'd322, 32'd323, 32'd324: capture_velocity<=16'd74;
                32'd325, 32'd326, 32'd327, 32'd328, 32'd329: capture_velocity<=16'd75;
                32'd330, 32'd331, 32'd332, 32'd333: capture_velocity<=16'd76;
                32'd334, 32'd335, 32'd336, 32'd337: capture_velocity<=16'd77;
                32'd338, 32'd339, 32'd340, 32'd341, 32'd342: capture_velocity<=16'd78;
                32'd343, 32'd344, 32'd345, 32'd346: capture_velocity<=16'd79;
                32'd347, 32'd348, 32'd349, 32'd350: capture_velocity<=16'd80;
                32'd351, 32'd352, 32'd353, 32'd354, 32'd355: capture_velocity<=16'd81;
                32'd356, 32'd357, 32'd358, 32'd359: capture_velocity<=16'd82;
                32'd360, 32'd361, 32'd362, 32'd363: capture_velocity<=16'd83;
                32'd364, 32'd365, 32'd366, 32'd367, 32'd368: capture_velocity<=16'd84;
                32'd369, 32'd370, 32'd371, 32'd372: capture_velocity<=16'd85;
                32'd373, 32'd374, 32'd375, 32'd376: capture_velocity<=16'd86;
                32'd377, 32'd378, 32'd379, 32'd380, 32'd381: capture_velocity<=16'd87;
                32'd382, 32'd383, 32'd384, 32'd385: capture_velocity<=16'd88;
                32'd386, 32'd387, 32'd388, 32'd389: capture_velocity<=16'd89;
                32'd390, 32'd391, 32'd392, 32'd393, 32'd394: capture_velocity<=16'd90;
                32'd395, 32'd396, 32'd397, 32'd398: capture_velocity<=16'd91;
                32'd399, 32'd400, 32'd401, 32'd402: capture_velocity<=16'd92;
                32'd403, 32'd404, 32'd405, 32'd406, 32'd407: capture_velocity<=16'd93;
                32'd408, 32'd409, 32'd410, 32'd411: capture_velocity<=16'd94;
                32'd412, 32'd413, 32'd414, 32'd415: capture_velocity<=16'd95;
                32'd416, 32'd417, 32'd418, 32'd419, 32'd420: capture_velocity<=16'd96;
                32'd421, 32'd422, 32'd423, 32'd424: capture_velocity<=16'd97;
                32'd425, 32'd426, 32'd427, 32'd428: capture_velocity<=16'd98;
                32'd429, 32'd430, 32'd431, 32'd432, 32'd433: capture_velocity<=16'd99;
                32'd434, 32'd435, 32'd436, 32'd437: capture_velocity<=16'd100;
                32'd438, 32'd439, 32'd440, 32'd441: capture_velocity<=16'd101;
                32'd442, 32'd443, 32'd444, 32'd445, 32'd446: capture_velocity<=16'd102;
                32'd447, 32'd448, 32'd449, 32'd450: capture_velocity<=16'd103;
                32'd451, 32'd452, 32'd453, 32'd454: capture_velocity<=16'd104;
                32'd455, 32'd456, 32'd457, 32'd458, 32'd459: capture_velocity<=16'd105;
                32'd460, 32'd461, 32'd462, 32'd463: capture_velocity<=16'd106;
                32'd464, 32'd465, 32'd466, 32'd467: capture_velocity<=16'd107;
                32'd468, 32'd469, 32'd470, 32'd471, 32'd472: capture_velocity<=16'd108;
                32'd473, 32'd474, 32'd475, 32'd476: capture_velocity<=16'd109;
                32'd477, 32'd478, 32'd479, 32'd480: capture_velocity<=16'd110;
                32'd481, 32'd482, 32'd483, 32'd484, 32'd485: capture_velocity<=16'd111;
                32'd486, 32'd487, 32'd488, 32'd489: capture_velocity<=16'd112;
                32'd490, 32'd491, 32'd492, 32'd493: capture_velocity<=16'd113;
                32'd494, 32'd495, 32'd496, 32'd497, 32'd498: capture_velocity<=16'd114;
                32'd499, 32'd500, 32'd501, 32'd502: capture_velocity<=16'd115;
                32'd503, 32'd504, 32'd505, 32'd506: capture_velocity<=16'd116;
                32'd507, 32'd508, 32'd509, 32'd510, 32'd511: capture_velocity<=16'd117;
                32'd512, 32'd513, 32'd514, 32'd515: capture_velocity<=16'd118;
                32'd516, 32'd517, 32'd518, 32'd519: capture_velocity<=16'd119;
                32'd520, 32'd521, 32'd522, 32'd523, 32'd524: capture_velocity<=16'd120;
                32'd525, 32'd526, 32'd527, 32'd528: capture_velocity<=16'd121;
                32'd529, 32'd530, 32'd531, 32'd532: capture_velocity<=16'd122;
                32'd533, 32'd534, 32'd535, 32'd536, 32'd537: capture_velocity<=16'd123;
                32'd538, 32'd539, 32'd540, 32'd541: capture_velocity<=16'd124;
                32'd542, 32'd543, 32'd544, 32'd545: capture_velocity<=16'd125;
                32'd546, 32'd547, 32'd548, 32'd549, 32'd550: capture_velocity<=16'd126;
                32'd551, 32'd552, 32'd553, 32'd554: capture_velocity<=16'd127;
                32'd555, 32'd556, 32'd557, 32'd558: capture_velocity<=16'd128;
                32'd559, 32'd560, 32'd561, 32'd562, 32'd563: capture_velocity<=16'd129;
                32'd564, 32'd565, 32'd566, 32'd567: capture_velocity<=16'd130;
                32'd568, 32'd569, 32'd570, 32'd571: capture_velocity<=16'd131;
                32'd572, 32'd573, 32'd574, 32'd575, 32'd576: capture_velocity<=16'd132;
                32'd577, 32'd578, 32'd579, 32'd580: capture_velocity<=16'd133;
                32'd581, 32'd582, 32'd583, 32'd584: capture_velocity<=16'd134;
                32'd585, 32'd586, 32'd587, 32'd588, 32'd589: capture_velocity<=16'd135;
                32'd590, 32'd591, 32'd592, 32'd593: capture_velocity<=16'd136;
                32'd594, 32'd595, 32'd596, 32'd597: capture_velocity<=16'd137;
                32'd598, 32'd599, 32'd600: capture_velocity<=16'd138;
                default: capture_velocity<=0;
            endcase
        end
    end

    reg signed [32:0] travel_pipe;
    reg [32:0] travel_magnitude;
    reg travel_exceeded;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin travel_pipe<=0; travel_magnitude<=0; travel_exceeded<=0; end
        else if(!request || state==IDLE) begin
            travel_pipe<=0; travel_magnitude<=0; travel_exceeded<=0;
        end else begin
            travel_pipe<=travel;
            travel_magnitude<=travel_pipe[32] ? -travel_pipe : travel_pipe;
            travel_exceeded<=travel_magnitude>{1'b0,travel_max};
        end
    end
    wire [31:0] step_abs = step_target[31] ? -step_target : step_target;
    wire config_ok = pulse_ms>=20 && pulse_ms<=1000 && timeout_ms>pulse_ms && timeout_ms<=5000 &&
                     travel_max>=60 && travel_max<=520 && switch_deg>=5 && switch_deg<=30 &&
                     switch_dps>=10 && switch_dps<=600 && pulse_duty>=-1024 && pulse_duty<=1024 &&
                     (pulse_duty<=-100 || pulse_duty>=100) && gains_valid &&
                     step_abs>=20 && step_abs<=400 && step_abs+32'd40<=travel_max &&
                     step_kp>0 && step_kp<=32'sd1048576 && step_kd>=0 && step_kd<=32'sd8388608 &&
                     capture_ms>=5 && capture_ms<=100 && hang>=200 && center>hang && center-hang>=1500;
    wire upright_raw = sensor_valid && !gap_active && valid_count==20 && angle_abs<=capture_angle && velocity_abs<=capture_velocity;

    reg upright;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) upright<=1'b0;
        else if(!request) upright<=1'b0;
        else upright<=upright_raw;
    end
    wire [15:0] max_drive = pulse_duty[31] ? -pulse_duty[15:0] : pulse_duty[15:0];

    wire signed [16:0] vel_raw  = $signed({1'b0,theta}) - $signed({1'b0,history[history_idx]});
    wire signed [16:0] vel_fold = vel_raw > 17'sd2048  ? vel_raw - 17'sd4096 :
                                  vel_raw < -17'sd2048 ? vel_raw + 17'sd4096 :
                                  vel_raw;

    wire signed [31:0] damp_gain = (state==REST) ? settle_gain : brake_gain;
    reg signed [31:0] damp_gain_r;
    reg signed [48:0] damp_prod_r;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin damp_gain_r<=32'sd0; damp_prod_r<=49'sd0; end
        else begin
            damp_gain_r<=damp_gain;
            damp_prod_r<=vel_fold * $signed(damp_gain_r);
        end
    end
    wire signed [32:0] damp_shr = damp_prod_r >>> 16;
    wire signed [16:0] damp_lim = $signed({1'b0, max_drive});
    wire signed [15:0] damp = damp_shr >  damp_lim ? $signed({1'b0, max_drive}) :
                              damp_shr < -damp_lim ? -$signed({1'b0, max_drive}) :
                              damp_shr[15:0];
    wire settle_zone = hang_abs <= 16'd1024;
    reg signed [15:0] local_pos;
    reg signed [31:0] actual_q16,target_q16,limit_q16;
    reg [2:0] pid_div;
    reg step_tick;
    reg [7:0] ready_pipe;
    reg drive_ready;
    wire signed [31:0] position_pid;
    wire step_clear = !request || state!=POSITION;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            local_pos<=0;actual_q16<=0;target_q16<=0;limit_q16<=0;
            pid_div<=0;step_tick<=0;ready_pipe<=0;drive_ready<=0;
        end else begin
            local_pos<=travel_pipe>1024 ? 16'sd1024 : travel_pipe< -1024 ? -16'sd1024 : travel_pipe[15:0];
            actual_q16<={local_pos,16'd0};

            target_q16<={swing_target[15:0],16'd0};
            limit_q16<={max_drive,16'd0};
            step_tick<=0;
            if(step_clear) begin pid_div<=0;ready_pipe<=0;drive_ready<=0;end
            else begin
                if(tick) begin
                    if(pid_div==4) begin pid_div<=0;step_tick<=1;end
                    else pid_div<=pid_div+1'b1;
                end
                ready_pipe<={ready_pipe[6:0],step_tick};
                if(ready_pipe[7]) drive_ready<=1;
            end
        end
    end

    pid_q16 u_position_pid(.clk(clk),.rst_n(rst_n),.en(step_tick),.clear(step_clear),
        .target(target_q16),.actual(actual_q16),.kp(step_kp),.ki(32'sd0),.kd(step_kd),
        .out_max(limit_q16),.out_min(-limit_q16),
        .en_dlpf(1'b0),.en_dspan2(1'b0),

        .en_dspan4(1'b0),.lpf_shift(2'd0),
        .out(position_pid));
    assign balance = request && (state==BALANCE || state==CAPTURE);
    assign telemetry_pos = travel_pipe>8191 ? 16'sd8191 :
                           travel_pipe< -8191 ? -16'sd8191 : travel_pipe[15:0];
    always @(*) begin
        drive=0;
        if(request && state==POSITION && drive_ready && config_ok && sensor_valid && !gap_allowed &&
           !travel_exceeded && dp_abs<=3)

            drive=-$signed(position_pid[31:16]);
        else if(request && state==WAIT_UP && config_ok && sensor_valid && !gap_allowed &&
                !travel_exceeded && dp_abs<=3)

            drive=damp;
        else if(request && state==REST && config_ok && sensor_valid && !gap_allowed &&
                !travel_exceeded && dp_abs<=3 && valid_count==20 && settle_zone)

            drive=damp;
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;elapsed<=0;stable<=0;origin<=0;history_idx<=0;valid_count<=0;previous<=0;
            fault_code<=0;entered<=0;confirmed<=0;gap_seen<=0;gap_active<=0;
            gap_age<=0;capture_age<=0;previous_raw<=0;retry_count<=2'd0;rest_count<=3'd0;
            swing_idx<=2'd0;
            jump_bad<=3'd0;
            for(k=0;k<20;k=k+1) history[k]<=0;
        end else if(!request) begin
            state<=IDLE;elapsed<=0;stable<=0;valid_count<=0;history_idx<=0;fault_code<=0;
            entered<=0;confirmed<=0;gap_seen<=0;gap_active<=0;gap_age<=0;capture_age<=0;previous_raw<=raw_angle;
            retry_count<=2'd0;rest_count<=3'd0;
            swing_idx<=2'd0;
            jump_bad<=3'd0;
        end else if(tick) begin

            previous<=theta;
            if(raw_trusted) previous_raw<=raw_angle;
            history[history_idx]<=theta;
            history_idx<=history_idx==19 ? 0 : history_idx+1'b1;

            if(jump_glitch) jump_bad<=(jump_bad==JUMP_DEBOUNCE-1'b1)?jump_bad:jump_bad+1'b1;
            else jump_bad<=3'd0;
            if(!sensor_valid || gap_allowed) valid_count<=0;
            else if(valid_count<20) valid_count<=valid_count+1'b1;
            if(state!=IDLE && state!=FAULT && state!=BALANCE) elapsed<=elapsed+1'b1;
            if(state==IDLE) begin
                origin<=position; elapsed<=0; stable<=0; state<=REST;
            end else if(state!=FAULT && ((!sensor_valid && !gap_allowed && state!=REST) ||
                        (jump_dead && !gap_allowed && state!=REST) ||
                        !sensor_fresh || !config_ok || (dp_abs>3 && state!=REST) ||
                        (state!=BALANCE && state!=REST && travel_exceeded))) begin

                state<=FAULT;

                if(!sensor_fresh || (!sensor_valid && !gap_allowed && state!=REST)) fault_code<=1;
                else if(!config_ok) fault_code<=2;
                else if(dp_abs>3) fault_code<=3;
                else if(jump_dead && !gap_allowed && state!=REST) fault_code<=4;
                else fault_code<=5;
            end else if(gap_allowed) begin

                state<=WAIT_UP;stable<=0;gap_seen<=1;
                if(gap_active && gap_age>=49) begin state<=FAULT;fault_code<=9;gap_active<=0;end
                else if(gap_active && sensor_valid && !near_edge && !edge_wrap) begin
                    gap_active<=0;gap_age<=0;
                end else begin gap_active<=1;gap_age<=gap_age+1'b1;end
            end else case(state)
                REST: begin

                    if(mode_handoff && upright) begin
                        state<=CAPTURE;stable<=0;capture_age<=0;entered<=1;
                    end
                    else if(elapsed>=1000) begin
                        if(rest_count<REST_MAX) begin
                            rest_count<=rest_count+1'b1;
                            state<=IDLE;elapsed<=0;stable<=0;
                        end
                        else begin state<=FAULT;fault_code<=6;end
                    end
                    else if(valid_count==20 && hang_abs<=100 && dp_abs<=1 && velocity_abs<=8) begin
                        if(stable==99) begin
                            state<=POSITION;elapsed<=0;stable<=0;

                            origin<=position;
                        end
                        else stable<=stable+1'b1;
                    end else stable<=0;
                end
                POSITION, WAIT_UP: begin
                    if(elapsed+1>=timeout_ms) begin state<=FAULT;fault_code<=7;end
                    else if(upright) begin
                        state<=CAPTURE;stable<=0;capture_age<=0;entered<=1;
                    end else begin
                        stable<=0;
                        if(state==POSITION && elapsed+1>=pulse_ms) begin
                            state<=WAIT_UP;

                            if(swing_idx<2'd3) swing_idx<=swing_idx+1'b1;
                        end
                    end
                end
                CAPTURE: begin
                    capture_age<=capture_age+1'b1;

                    if(balance_safe || angle_abs>700) begin state<=FAULT;fault_code<=8;end
                    else if(capture_age>=499) begin state<=FAULT;fault_code<=10;end
                    else if(upright) begin
                        if({24'd0,stable}+32'd1>=capture_ms) begin
                            state<=BALANCE;confirmed<=1;stable<=0;

                            retry_count<=2'd0;
                            swing_idx<=2'd0;
                        end else stable<=stable+1'b1;
                    end else stable<=0;
                end
                BALANCE: if(balance_safe) begin state<=FAULT;fault_code<=8;end
                FAULT: begin

                    if(retryable && retry_count<RETRY_MAX) begin
                        retry_count<=retry_count+1'b1;
                        state<=IDLE;elapsed<=0;stable<=0;valid_count<=0;history_idx<=0;
                        fault_code<=0;entered<=0;confirmed<=0;gap_seen<=0;gap_active<=0;
                        gap_age<=0;capture_age<=0;

                        if(raw_trusted) previous_raw<=raw_angle;
                        rest_count<=3'd0;jump_bad<=3'd0;
                    end else begin
                        state<=FAULT;gap_active<=0;
                    end
                end
                default: state<=FAULT;
            endcase
        end
    end
endmodule
