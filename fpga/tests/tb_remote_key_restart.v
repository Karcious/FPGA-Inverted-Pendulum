`timescale 1ns/1ps
module tb_remote_key_restart;
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

        params[47*32+:32]=16384;
        params[16*32+:32]=3;force dut.u_swing.state=3'd4;
        force dut.curve_wr_en=bus_en;force dut.curve_wr_addr=bus_addr;force dut.curve_wr_data=bus_data;
        ct;ct;wr(79,1);
        check(!dut.key_command_reply[7] && dut.key_command_reply[6:4]==4,"not-ready command is rejected explicitly");
        outers(30);check(dut.motion_ready,"initial ready");

        force dut.u_swing.state=3'd6;encoder=10050;outers(5);
        force dut.u_swing.state=3'd4;
        for(i=0;i<1000;i=i+1)begin
            encoder=10000+($signed(dut.u_key_motion.target_ref)>>>16);ct;
        end
        check(dut.motion_ready,"restart with ideal zero tracking becomes ready");

        force dut.u_swing.state=3'd6;encoder=10050;outers(5);
        force dut.u_swing.state=3'd4;
        for(i=0;i<1000;i=i+1)begin
            encoder=10003+($signed(dut.u_key_motion.target_ref)>>>16);ct;
        end
        wr(79,1);check(dut.key_command_reply[7],"command says accepted");
        check(dut.u_key_motion.target_cmd==-(260<<16),"accepted command actually changes target after restart");
        outers(90);
        $display("DIAGNOSTIC ready=%b homing=%b home_fault=%b pending=%d ref=%d target=%d pos=%d receipt=%h status=%h",
          dut.motion_ready,dut.motion_homing,dut.motion_home_fault,dut.u_key_motion.pending_key,
          dut.u_key_motion.target_ref>>>16,dut.u_key_motion.target_cmd>>>16,dut.pos_off,
          dut.key_command_reply,dut.control_status_q);
        check(dut.motion_ready,"small static error must not permanently block restart key commands");

        force dut.u_swing.state=3'd6;encoder=10050;outers(5);
        force dut.u_swing.state=3'd4;
        for(i=0;i<1000;i=i+1)begin
            encoder=10008+($signed(dut.u_key_motion.target_ref)>>>16);ct;
        end
        check(!dut.motion_ready,"eight-count residual still waits for settling");
        dut.u_key_motion.home_count=29999;ct;
        check(dut.motion_home_fault,"real timeout remains a fault");
        wr(79,2);check(!dut.key_command_reply[7],"home fault rejects software motion");
        @(negedge clk);force dut.zero_long_press=1'b1;
        @(negedge clk);release dut.zero_long_press;
        outers(12);
        check(!dut.motion_home_fault && dut.motion_ready && dut.pos_off==0,"SW3 long press recovers readiness at current balance position");
        wr(79,2);check(dut.key_command_reply[7] && dut.u_key_motion.target_cmd==(260<<16),"right movement works after fault recovery");
        $display("ALL PASS remote key restart %0d",checks);$finish;
    end
endmodule
