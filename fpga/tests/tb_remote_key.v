`timescale 1ns/1ps
module tb_remote_key;
    wire pwm,an1,an2,pwm_b,bn1,bn2,adc_clk,adc_oe,tx;
    wire [3:0] led;

    localparam [31:0] SENT = 32'sd777 <<< 16;

    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,tick=0,key=1,seen_outer=0;

    reg [52*32-1:0] params=0;
    reg [15:0] theta=3095;
    reg signed [31:0] encoder=10000;
    reg signed [15:0] delta=0;
    integer checks=0, i, r0;

    pend_top dut(.sw1_n(key),.sw2_n(1'b1),.sw3_n(1'b1),.sw4_n(1'b1),
        .clk(clk),.rst_n(rst_n),.uart_rx_pin(1'b1),.uart_tx_pin(tx),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe),.adc_data_in(10'd740),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.pwm(pwm),.an1(an1),.an2(an2),
        .pwm_b(pwm_b),.bn1(bn1),.bn2(bn2),.led(led));

    task check; input ok; input [8*64-1:0] name; begin
        checks=checks+1;
        if(ok!==1) $fatal(1,"spin [%0s] chk=%0d ref=%0d latch=%b ang=%0d vel=%0d act=%b",
            name,checks,$signed(dut.spin_ref),dut.spin_tilt_latch,
            $signed(dut.spin_angle_abs),$signed(dut.vel_sum),dut.spin_active);
    end endtask

    task ct; begin
        @(negedge clk); tick=1;
        @(posedge clk); #1;
        if (dut.outer_tick===1'b1) seen_outer=1'b1;
        @(negedge clk); tick=0;
        repeat(40)@(negedge clk);
    end endtask

    task outers; input integer n; integer j; begin
        for(j=0;j<25*n;j=j+1) ct;
    end endtask

    task align_outer; integer j; begin
        seen_outer=1'b0; j=0;
        while(!seen_outer && j<60) begin ct; j=j+1; end
        if(!seen_outer) $fatal(1,"no outer tick within 60 ctrl ticks");
    end endtask

    reg bus_en=0;reg[7:0]bus_addr=0;reg[31:0]bus_data=0;
    task wr;input[7:0]a;input[31:0]v;begin
      @(negedge clk);bus_addr=a;bus_data=v;bus_en=1;
      @(negedge clk);bus_en=0;repeat(5)@(negedge clk);
    end endtask
    initial begin
        check(dut.OUTER_DIV==25,"outer period is 25ms");

        check(dut.u_dbg.u_proto.u_params.MAGIC==32'h0002_0719,"0x0207 identity");
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

        params[40*32+:32]=32'h0000_2000;params[41*32+:32]=32'd3787;params[42*32+:32]=32'd0;
        params[43*32+:32]=32'd10;params[44*32+:32]=-(8<<16);params[45*32+:32]=32'd0;
        params[46*32+:32]=32'd4;
        #1;
        repeat(4)@(negedge clk);rst_n=1;ct;ct;

        params[16*32+:32]=3;force dut.u_swing.state=3'd4;
        force dut.curve_wr_en=bus_en;force dut.curve_wr_addr=bus_addr;force dut.curve_wr_data=bus_data;
        outers(30);
        check(dut.motion_ready,"ready before software keys");
        wr(79,1);check(dut.key_command_reply==32'h181,"left acknowledged once");
        check(dut.u_key_motion.target_cmd==-(260<<16),"software SW2 target minus90");
        r0=encoder;
        for(i=0;i<5000 && dut.u_position.origin==r0;i=i+1)begin
            encoder=r0+($signed(dut.u_key_motion.target_ref)>>>16);ct;
        end
        check(dut.u_position.origin!=r0 && dut.pos_off==0,"left arrives and rebases actual endpoint");
        r0=encoder;wr(79,2);
        check(dut.key_command_reply==32'h282,"right acknowledged once");
        check(dut.u_key_motion.target_cmd==(260<<16),"software SW4 plus90 from new zero");
        for(i=0;i<5000 && dut.u_position.origin==r0;i=i+1)begin
            encoder=r0+($signed(dut.u_key_motion.target_ref)>>>16);ct;
        end
        check(dut.u_position.origin!=r0 && dut.pos_off==0,"right arrives and rebases actual endpoint");
        repeat(10)ct;check(dut.key_command_reply==32'h282,"no automatic command repeats");
        wr(79,99);check(dut.key_command_reply==32'h333,"invalid command rejected");
        force dut.curve_owned=1;wr(79,1);
        check(dut.key_command_reply==32'h421,"curve owns controller: reject key");
        check(!dut.remote_key_neg&&!dut.remote_key_pos,"rejection emits no pulse");
        release dut.curve_owned;
        params[16*32+:32]=0;ct;ct;wr(79,2);
        check(dut.key_command_reply==32'h512,"stopped: reject key");
        check(dut.u_key_motion.pending_key==0,"stopped command cannot linger");
        params[16*32+:32]=3;ct;ct;
        force dut.safe=1;wr(79,1);
        check(dut.key_command_reply==32'h611,"protection: reject key");
        release dut.safe;

        check(!dut.zero_marked,"no zero yet");

        force dut.safe=1;repeat(3)ct;
        wr(79,3);
        check(dut.key_command_reply==32'h753 && !dut.zero_marked,
              "zero rejected with reason 5, zero untouched");
        release dut.safe;

        params[16*32+:32]=0;repeat(3)ct;
        wr(79,3);
        check(dut.key_command_reply==32'h883 && dut.zero_marked,
              "zero accepted while coasting");
        check(dut.curve_readouts[480+:32]==dut.key_command_reply,"registered readout returns command receipt");

        params[16*32+:32]=3;ct;ct;
        outers(30);
        check(dut.motion_ready,"ready again after the remote zero mark");
        check(dut.u_key_motion.arrival_count==8'd2,"two key arrivals counted so far");

        wr(79,{16'h005A,16'd4});
        check(dut.key_command_reply==32'h984,"cmd4 +90 counts acknowledged");
        check(dut.u_key_motion.target_cmd==(90<<16),"cmd4 target = +90 counts");
        r0=encoder;
        for(i=0;i<5000 && dut.u_position.origin==r0;i=i+1)begin
            encoder=r0+($signed(dut.u_key_motion.target_ref)>>>16);ct;
        end
        check(dut.u_position.origin!=r0 && dut.pos_off==0,"cmd4 +90 arrives and rebases zero");
        check(dut.u_key_motion.arrival_count==8'd3,"arrival count ticks up exactly once");
        check(dut.u_key_motion.arrival_error==16'sd0,"arrival error = measured - target");

        wr(79,{16'hFFA6,16'd4});
        check(dut.key_command_reply==32'hA84,"cmd4 -90 counts acknowledged");
        check(dut.u_key_motion.target_cmd==-(90<<16),"cmd4 target = -90 counts from the new zero");
        r0=encoder;
        for(i=0;i<5000 && dut.u_position.origin==r0;i=i+1)begin
            encoder=r0+($signed(dut.u_key_motion.target_ref)>>>16);ct;
        end
        check(dut.u_position.origin!=r0 && dut.pos_off==0,"cmd4 -90 arrives and rebases zero");
        check(dut.u_key_motion.arrival_count==8'd4,"fourth arrival counted");

        r0=dut.u_key_motion.target_cmd;
        wr(79,{16'd0,16'd4});
        check(dut.key_command_reply==32'hB34,"cmd4 zero counts rejected (reason 3)");
        check(dut.u_key_motion.target_cmd==r0,"rejected command does not move the target");
        wr(79,{16'h2000,16'd4});
        check(dut.key_command_reply==32'hC34,"cmd4 beyond 8191 counts rejected (reason 3)");
        check(dut.u_key_motion.target_cmd==r0,"still no movement from a rejected command");

        force dut.curve_owned=1;wr(79,{16'h001E,16'd4});
        check(dut.key_command_reply==32'hD24,"curve owns controller: reject cmd4 (reason 2)");
        release dut.curve_owned;
        params[16*32+:32]=0;ct;ct;wr(79,{16'h001E,16'd4});
        check(dut.key_command_reply==32'hE14,"stopped: reject cmd4 (reason 1)");
        params[16*32+:32]=3;ct;ct;

        repeat(3)@(negedge clk);
        $display("  addr75=%08h err=%0d cnt=%0d parked=%b",
                 dut.curve_readouts[352+:32],$signed(dut.u_key_motion.arrival_error),
                 dut.u_key_motion.arrival_count,dut.spin_parked);
        check(dut.curve_readouts[352+:32]==32'h0000_0400,
              "addr75 = {error=0, count=4, parked=0}");
        check(dut.curve_readouts[352+:32]==={dut.u_key_motion.arrival_error,
                                            dut.u_key_motion.arrival_count,7'd0,dut.spin_parked},
              "addr75 layout matches the frozen fields");

        params[43*32+:32]=32'd10;outers(30);
        check(dut.motion_ready,"cmd5: ready after the previous checks");
        wr(79,{16'd0,16'd5});
        check(dut.key_command_reply==32'h0000_0F35,"cmd5 zero counts rejected (reason 3)");
        check(!dut.travel_mode,"cmd5 rejection does not arm the move");
        params[43*32+:32]=32'd0;ct;ct;ct;
        wr(79,{16'h0032,16'd5});
        check(dut.key_command_reply==32'h0000_1055,"cmd5 without a speed rejected (reason 5)");
        check(!dut.travel_mode,"cmd5 without speed does not arm the move");
        params[43*32+:32]=32'd10;ct;ct;
        force dut.curve_owned=1;wr(79,{16'h0032,16'd5});
        check(dut.key_command_reply==32'h0000_1125,"cmd5 curve owns: reason 2");
        release dut.curve_owned;
        params[16*32+:32]=0;ct;ct;wr(79,{16'h0032,16'd5});
        check(dut.key_command_reply==32'h0000_1215,"cmd5 stopped: reason 1");
        params[16*32+:32]=3;ct;ct;
        force dut.travel_mode=1'b1;force dut.travel_done=1'b0;
        wr(79,{16'h0032,16'd5});
        check(dut.key_command_reply==32'h0000_1315,"cmd5 while another move runs: reason 1");
        release dut.travel_mode;release dut.travel_done;

        force dut.travel_mode=1'b0;force dut.travel_finish=1'b0;force dut.travel_done=1'b0;
        ct;ct;
        release dut.travel_mode;release dut.travel_finish;release dut.travel_done;

        outers(30);
        check(dut.motion_ready,"cmd5: ready recovered before the accept");
        wr(79,{16'h012C,16'd5});
        check(dut.key_command_reply==32'h0000_1485,"cmd5 +300 counts acknowledged");
        ct;ct;
        check(dut.travel_mode && dut.travel_target===32'sd300,
              "cmd5 arms a 300-count travel");
        check(dut.spin_active,"cmd5 uses the velocity loop");
        $display("ALL PASS remote key %0d",checks);$finish;
    end
endmodule
