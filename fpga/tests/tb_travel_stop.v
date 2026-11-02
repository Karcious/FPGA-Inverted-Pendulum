`timescale 1ns/1ps

module tb_travel_stop;
    wire pwm,an1,an2,pwm_b,bn1,bn2,adc_clk,adc_oe,tx;
    wire [3:0] led;

    localparam [31:0] SENT = 32'sd777 <<< 16;
    localparam integer LAG = 2;

    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,tick=0,key=1,seen_outer=0;
    reg [52*32-1:0] params=0;
    reg [15:0] theta=3095;
    reg signed [31:0] encoder=10000;
    reg signed [15:0] delta=0;
    integer checks=0, i;

    integer plant_v=0, frac=0;

    pend_top dut(.sw1_n(key),.sw2_n(1'b1),.sw3_n(1'b1),.sw4_n(1'b1),
        .clk(clk),.rst_n(rst_n),.uart_rx_pin(1'b1),.uart_tx_pin(tx),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe),.adc_data_in(10'd740),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.pwm(pwm),.an1(an1),.an2(an2),
        .pwm_b(pwm_b),.bn1(bn1),.bn2(bn2),.led(led));

    task check; input ok; input [8*72-1:0] name; begin
        checks=checks+1;
        if(ok!==1) $fatal(1,"travel_stop [%0s] chk=%0d prog=%0d tgt=%0d left=%0d v=%0d plant=%0d done=%b",
            name,checks,$signed(dut.travel_progress),$signed(dut.travel_target),
            $signed(dut.travel_left),$signed(dut.travel_v),plant_v,dut.travel_done);
    end endtask

    task plant_ms; begin
        frac = frac + plant_v;
        while (frac >= 25)  begin frac = frac - 25; delta = delta + 16'sd1; encoder = encoder + 1; end
        while (frac <= -25) begin frac = frac + 25; delta = delta - 16'sd1; encoder = encoder - 1; end
    end endtask

    task ct; begin
        @(negedge clk); plant_ms; tick=1;
        @(posedge clk); #1;
        if (dut.outer_tick===1'b1) begin
            seen_outer=1'b1;

            if (($signed(dut.spin_ref) - plant_v) >= LAG || (plant_v - $signed(dut.spin_ref)) >= LAG)
                plant_v = plant_v + ($signed(dut.spin_ref) - plant_v)/LAG;
            else plant_v = $signed(dut.spin_ref);
        end
        @(negedge clk); tick=0; delta=0;
    end endtask

    task outers; input integer n; integer j; begin
        for(j=0;j<25*n;j=j+1) ct;
    end endtask
    task align_outer; integer j; begin
        seen_outer=1'b0; j=0;
        while(!seen_outer && j<60) begin ct; j=j+1; end
        if(!seen_outer) $fatal(1,"no outer tick within 60 ctrl ticks");
    end endtask

    task travel_req; input signed [15:0] counts; begin
        force dut.travel_req_r=1'b1; force dut.travel_counts_r=counts;
        @(negedge clk); @(negedge clk);
        release dut.travel_req_r; release dut.travel_counts_r;
        ct; ct;
    end endtask

    task run_to_park; integer n; begin
        n=0;
        while(!(dut.spin_parked && dut.travel_done) && n<12000) begin ct; n=n+1; end
        if(n>=12000) $fatal(1,"park never happened: prog=%0d left=%0d v=%0d",
            $signed(dut.travel_progress),$signed(dut.travel_left),$signed(dut.travel_v));
    end endtask

    initial begin
        force dut.param_flat=params;
        force dut.ctrl_tick=tick;
        force dut.theta=theta;
        force dut.enc_pos=encoder;
        force dut.enc_delta=delta;
        force dut.adc_valid=1'b1;
        force dut.theta_raw=12'd2960;
        force dut.enc_fault_seen=1'b0;
        force dut.pos_q16=SENT;
        params[3*32+:32]=3<<16;params[4*32+:32]=30<<16;
        params[5*32+:32]=-78643;params[20*32+:32]=-(24<<16);
        params[11*32+:32]=1024;params[19*32+:32]=2;params[18*32+:32]=3095;
        params[7*32+:32]=15;params[8*32+:32]=120;
        params[12*32+:32]=4<<16;params[14*32+:32]=104858;
        params[24*32+:32]=-600;params[25*32+:32]=600;params[26*32+:32]=1000;
        params[29*32+:32]=341;params[30*32+:32]=10;
        params[27*32+:32]=450;params[28*32+:32]=1017;
        params[9*32+:32]=7;
        params[40*32+:32]=32'h0000_2000;params[41*32+:32]=32'd3787;params[42*32+:32]=32'd0;
        params[43*32+:32]=32'd60;params[44*32+:32]=-(8<<16);params[45*32+:32]=32'd0;
        params[46*32+:32]=32'd4;
        #1;
        repeat(4)@(negedge clk);rst_n=1;
        params[16*32+:32]=32'd3;
        force dut.u_swing.state=3'd4;
        ct; ct; ct;
        check(dut.motion_active && !dut.travel_mode,"starts idle in mode 3");

        travel_req(16'sd1560);
        check(dut.travel_mode && !dut.travel_done,"540 travel armed");
        run_to_park;
        $display("① 540°: progress=%0d err=%0d codes (%.1f deg)",
                 $signed(dut.travel_progress),$signed(dut.travel_progress)-1560,
                 ($signed(dut.travel_progress)-1560)*360.0/1040.0);
        check($signed(dut.travel_progress)>=1560-40,"540 lands on the target (not short)");
        check($signed(dut.travel_progress)<=1560+40,"540 does not overshoot (REV16 was +480)");
        check(dut.travel_count===8'd1,"540 counted once");
        check(dut.u_position.origin===encoder,"540 park rebases the origin");
        check(!dut.spin_active,"540 park releases the velocity loop");
        check(dut.outer_target_sel===32'sd0,"540 park: target is the new zero");

        travel_req(16'sd520);
        check(dut.travel_mode && !dut.travel_done,"520 travel armed");
        ct; ct; outers(2);
        check($signed(dut.travel_progress)<520-100,"520 actually moves (not an instant finish)");
        run_to_park;
        $display("② 520: progress=%0d err=%0d codes",$signed(dut.travel_progress),
                 $signed(dut.travel_progress)-520);
        check($signed(dut.travel_progress)>=520-40,"520 lands on the target");
        check($signed(dut.travel_progress)<=520+40,"520 does not overshoot");
        check(dut.travel_count===8'd2,"520 counted once more");

        theta=16'd3095+16'd120;
        travel_req(16'sd260);
        run_to_park;
        check(dut.travel_done,"tilted 120: park still completes");
        check(dut.u_position.origin===encoder,"tilted 120: origin still rebased");
        theta=16'd3095;

        travel_req(16'sd14);
        run_to_park;
        check($signed(dut.travel_progress)>0,"5° travel actually moves");
        check($signed(dut.travel_progress)<=14+16,"5° travel does not run away");

        travel_req(16'sd900);
        outers(6);
        check(dut.travel_mode && !dut.travel_done,"abandon: travel running");
        plant_v=0; frac=0;
        i=$signed(encoder);
        release dut.u_swing.state;
        force dut.u_swing.state=3'd0;
        repeat(6)@(negedge clk); ct; ct;
        check(!dut.motion_active,"abandon: balance lost");
        check(!dut.travel_mode,"abandon: travel dropped");
        encoder=i+300; ct; ct; ct;
        force dut.u_swing.state=3'd4;
        repeat(4)@(negedge clk); ct; ct; ct;
        check(dut.motion_active,"abandon: balance back");
        check(dut.u_position.origin===encoder,"abandon: origin rebased on re-entry");
        check(dut.position_target_r===32'sd0,"abandon: target cleared too");

        release dut.u_swing.state;
        release dut.pos_q16;
        $display("ALL PASS travel_stop checks=%0d",checks);$finish;
    end
    initial begin #40000000;$fatal(1,"timeout");end
endmodule
