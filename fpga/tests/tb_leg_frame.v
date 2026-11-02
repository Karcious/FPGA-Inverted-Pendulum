`timescale 1ns/1ps

module tb_leg_frame;
    wire pwm,an1,an2,pwm_b,bn1,bn2,adc_clk,adc_oe,tx;
    wire [3:0] led;

    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,tick=0,key=1,sw2=1;

    reg [52*32-1:0] params=0;
    reg [15:0] theta=3095;
    reg signed [31:0] encoder=10000;
    reg signed [15:0] delta=0;
    integer checks=0, i, r0, origin0, travelled, steps;
    reg origin_moved, poked, poke_begun;

    pend_top dut(.sw1_n(key),.sw2_n(sw2),.sw3_n(1'b1),.sw4_n(1'b1),
        .clk(clk),.rst_n(rst_n),.uart_rx_pin(1'b1),.uart_tx_pin(tx),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe),.adc_data_in(10'd740),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.pwm(pwm),.an1(an1),.an2(an2),
        .pwm_b(pwm_b),.bn1(bn1),.bn2(bn2),.led(led));

    task check; input ok; input [8*72-1:0] name; begin
        checks=checks+1;
        if(ok!==1) $fatal(1,"leg [%0s] chk=%0d origin=%0d trip=%0d legpos=%0d err=%0d poserr=%0d",
            name,checks,dut.u_position.origin,encoder-r0,
            $signed(dut.u_position.leg_pos),
            $signed(dut.u_key_motion.arrival_error),$signed(dut.position_error));
    end endtask

    task ct; begin
        @(negedge clk); tick=1;
        @(posedge clk); #1;
        @(negedge clk); tick=0;
        repeat(40)@(negedge clk);
    end endtask
    task outers; input integer n; integer j; begin
        for(j=0;j<25*n;j=j+1) ct;
    end endtask

    reg bus_en=0;reg[7:0]bus_addr=0;reg[31:0]bus_data=0;
    task wr;input[7:0]a;input[31:0]v;begin
      @(negedge clk);bus_addr=a;bus_data=v;bus_en=1;
      @(negedge clk);bus_en=0;repeat(5)@(negedge clk);
    end endtask

    task run_leg; integer j; input poke; begin
        j=0; origin_moved=1'b0; poked=1'b0; poke_begun=1'b0;
        while (j<30000 && dut.u_key_motion.travel_phase===1'b1) begin
            encoder = r0 + ($signed(dut.u_key_motion.target_ref)>>>16);
            ct; j=j+1;

            if (poke && !poke_begun && j>=2000)
                poke_begun = dut.u_key_motion.leg_active;
            if (poke_begun) begin
                if (j==2000)      sw2=1'b0;
                else if (j==2030) sw2=1'b1;
                else if (j==2060 && !poked) begin
                    poked=1'b1;
                    check(dut.u_key_motion.target_cmd===(1560<<16) &&
                          dut.u_key_motion.target_ref>32'sd0,
                          "physical SW2 mid-leg does not steal the leg");
                    check(dut.u_key_motion.leg_long===1'b1 && dut.u_position.leg_active===1'b1,
                          "leg frame survives a mid-leg physical key press");
                    wr(79,{16'd400,16'd4});
                    check(dut.key_command_reply[7]===1'b0,
                          "mid-leg command 4 is refused, not silently eaten");
                    check(dut.key_command_reply[6:4]===3'd1,"mid-leg command 4 says reason 1");
                    wr(79,1);
                    check(dut.key_command_reply[7]===1'b0,
                          "mid-leg key 90 is refused, not silently eaten");
                    check(dut.u_key_motion.target_cmd===(1560<<16),
                          "leg target untouched by the refused remote commands");
                end
            end

            if (dut.u_key_motion.travel_phase===1'b1 &&
                dut.u_position.origin !== origin0) origin_moved=1'b1;
        end
        steps=j;
        repeat(6) ct;
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

        params[3*32+:32]=3<<16;params[4*32+:32]=30<<16;
        params[5*32+:32]=-78643;params[20*32+:32]=-(24<<16);
        params[11*32+:32]=1024;params[19*32+:32]=2;params[18*32+:32]=3095;
        params[7*32+:32]=15;params[8*32+:32]=120;
        params[12*32+:32]=4<<16;params[14*32+:32]=104858;
        params[24*32+:32]=-600;params[25*32+:32]=600;params[26*32+:32]=1000;
        params[29*32+:32]=341;params[30*32+:32]=10;
        params[27*32+:32]=450;params[28*32+:32]=1017;
        params[9*32+:32]=7;
        params[40*32+:32]=32'h0000_2000;
        params[43*32+:32]=32'd10;params[44*32+:32]=-(8<<16);
        params[45*32+:32]=32'd0;params[46*32+:32]=32'd4;
        #1;
        repeat(4)@(negedge clk);rst_n=1;ct;ct;

        params[16*32+:32]=3;
        force dut.u_swing.state=3'd4;
        force dut.curve_wr_en=bus_en;force dut.curve_wr_addr=bus_addr;
        force dut.curve_wr_data=bus_data;
        outers(30);
        check(dut.motion_ready,"ready before the long leg");

        r0 = encoder + 37;
        encoder = r0;
        repeat(3) ct;
        origin0 = dut.u_position.origin;
        check(dut.u_position.disp_pipe>30 && dut.u_position.disp_pipe<45,
              "fixture: arm sits 37 counts off the zero");

        wr(79,{16'd1560,16'd4});
        check(dut.key_command_reply[7]===1'b1,"540 deg acknowledged");
        check(dut.key_command_reply[6:4]===3'd0,"540 deg no rejection reason");
        check(dut.u_key_motion.target_cmd===(1560<<16),"target = 1560 counts");
        check(dut.u_key_motion.target_ref===32'sd0,
              "reference restarts at 0 in the leg frame");
        check(dut.u_key_motion.leg_long===1'b1,"long leg flagged");
        check(dut.u_key_motion.leg_active===1'b1,"key controller is in the leg frame");
        check(dut.u_position.leg_active===1'b1,"position_feedback sees the leg");

        check(dut.u_position.leg_base===(r0-origin0),
              "leg base latched to the start displacement (37)");

        check(dut.u_position.leg_pos>=-2 && dut.u_position.leg_pos<=2,
              "no phantom step on the first leg cycle");
        check(dut.position_error<=(2<<16) && dut.position_error>=-(2<<16),
              "outer position error is zero at the leg start");

        run_leg(1);
        check(poked,"the mid-leg poke really happened");
        check(!origin_moved,"origin never rebases while the leg is active");
        check(steps<20000,"leg finished inside the timeout");
        check(dut.u_position.origin!==origin0,"leg end rebases the zero to the stop point");
        travelled = encoder - r0;
        check(travelled>=1558 && travelled<=1562,"travelled 1560 counts (not a folded 520)");
        check(dut.u_key_motion.leg_long===1'b0 && dut.u_key_motion.leg_active===1'b0,
              "leg flags cleared on arrival");
        check(dut.u_key_motion.target_cmd===32'sd0 && dut.u_key_motion.target_ref===32'sd0,
              "references cleared on arrival");
        check(dut.pos_off===16'sd0,"folded readout is zero after the rebase");
        check(dut.u_key_motion.arrival_error>=-3 && dut.u_key_motion.arrival_error<=3,
              "arrival error is in leg counts (~0), not a folded -1040");
        check(dut.position_error<=(3<<16) && dut.position_error>=-(3<<16),
              "outer error continuous across the arrival rebase");

        r0 = encoder;
        origin0 = dut.u_position.origin;
        wr(79,{16'd400,16'd4});
        check(dut.key_command_reply[7]===1'b1,"400 counts acknowledged");
        check(dut.u_key_motion.leg_long===1'b0,"400 counts stays in the folded frame");
        check(dut.u_key_motion.leg_active===1'b0,"folded leg does not raise leg_active");
        check(dut.u_position.leg_active===1'b0,"position_feedback is on the old path");
        run_leg(0);
        travelled = encoder - r0;
        check(travelled>=398 && travelled<=402,"folded leg travelled 400 counts");
        check(dut.u_key_motion.leg_long===1'b0,"folded leg still not flagged afterwards");

        r0 = encoder;
        origin0 = dut.u_position.origin;
        wr(79,{16'd520,16'd4});
        check(dut.key_command_reply[7]===1'b1,"520 counts acknowledged");
        check(dut.u_key_motion.leg_long===1'b0,"520 = last folded count");
        run_leg(0);

        r0 = encoder;
        origin0 = dut.u_position.origin;
        wr(79,{16'd521,16'd4});
        check(dut.key_command_reply[7]===1'b1,"521 counts acknowledged");
        check(dut.u_key_motion.leg_long===1'b1,"521 = first leg-frame count");
        run_leg(0);
        check(!origin_moved,"521-count leg also ran with the rebase gate closed");
        travelled = encoder - r0;
        check(travelled>=519 && travelled<=523,"521-count leg travelled 521 counts");

        r0 = encoder;
        origin0 = dut.u_position.origin;
        wr(79,1);
        check(dut.key_command_reply[7]===1'b1,"key 90 accepted after a long leg");
        check(dut.u_key_motion.target_cmd===-(260<<16),
              "key 90 is -260 from the new zero");
        check(dut.u_key_motion.leg_long===1'b0,"key leg is never a long leg");
        run_leg(0);
        check(dut.u_position.origin!==origin0,"key 90 rebases the zero");
        check(dut.pos_off===16'sd0,"key 90 ends on the new zero");

        $display("ALL PASS leg frame %0d",checks);$finish;
    end
endmodule
