`timescale 1ns/1ps

module tb_recovery_capture;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, tick=0;
    reg [52*32-1:0] params=0;
    reg [15:0] theta=3086;
    reg signed [31:0] encoder=10000;
    integer checks=0, k;
    wire tx, adc_clk, adc_oe, pwm, an1, an2, pwm_b, bn1, bn2;
    wire [3:0] led;
    pend_top dut(.clk(clk),.rst_n(rst_n),.sw1_n(1'b1),.sw2_n(1'b1),
        .sw3_n(1'b1),.sw4_n(1'b1),.uart_rx_pin(1'b1),.uart_tx_pin(tx),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe),.adc_data_in(10'd740),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.pwm(pwm),.an1(an1),.an2(an2),
        .pwm_b(pwm_b),.bn1(bn1),.bn2(bn2),.led(led));
    task step; begin
        @(negedge clk);tick=1;
        @(negedge clk);tick=0;
        repeat(62)@(negedge clk);
    end endtask
    task check; input ok; input [8*120-1:0] message; begin
        checks=checks+1;
        if(ok!==1) $fatal(1,"%0s outer=%0d angle=%0d duty=%0d pwm=%0d state=%0d clear=%b",
            message,dut.outer_out>>>16,dut.ang_out>>>16,dut.duty_cmd,
            dut.duty_applied,dut.swing_state,dut.position_pid_clear);
    end endtask
    task disturb_then_capture;
        input integer direction;
        input integer stop_first;
        begin
            params[16*32+:32]=0;
            theta=3086;
            repeat(8)step;
            params[16*32+:32]=2;
            repeat(5)step;
            encoder=encoder+direction*500;
            repeat(100)step;
            check(dut.outer_out==direction*(750<<16),
                  "disturbance must first saturate the actual position PID");
            if(stop_first) begin
                params[16*32+:32]=0;
                repeat(80)step;
                check(dut.duty_cmd==0,"SW1 stop must remain stopped");
            end
            params[16*32+:32]=3;
            force dut.u_swing.state=3'd6;

            theta=3086-direction*400;
            repeat(120)step;
            check(dut.mode_close && !dut.outer_run && dut.position_pid_clear,
                  "fixture must be in CAPTURE with position sampling disabled");
            check(dut.outer_out==0,"stale position demand must be erased during CAPTURE");
            check(dut.ang_out==direction*(1024<<16),"angle PID must reach its existing limit");
            check(dut.duty_cmd==direction*1024,
                  "CAPTURE must retain full angle authority instead of 274 raw / 404 PWM");
            check(dut.duty_applied==direction*1024,"motor demand must reach the existing full scale");

            theta=3086;
            force dut.u_swing.state=3'd4;
            repeat(3)step;
            check(dut.outer_out==0,"resuming BALANCE must not restore the pre-fall output");
            repeat(80)step;
            check(dut.outer_run && !dut.position_pid_clear,
                  "position feedback must resume after confirmed BALANCE");
            check(!dut.safe,"the existing fall protection must stay satisfied");
            release dut.u_swing.state;
            params[16*32+:32]=0;
            repeat(8)step;
        end
    endtask
    initial begin
        force dut.param_flat=params;
        force dut.ctrl_tick=tick;
        force dut.theta=theta;
        force dut.enc_pos=encoder;
        force dut.enc_delta=16'sd0;
        force dut.adc_valid=1'b1;
        force dut.theta_raw=12'd2951;
        force dut.enc_fault_seen=1'b0;
        params[3*32+:32]=229376;params[4*32+:32]=100<<16;
        params[5*32+:32]=-98304;params[6*32+:32]=0;
        params[9*32+:32]=15;params[10*32+:32]=130;
        params[11*32+:32]=1024;params[18*32+:32]=3086;params[19*32+:32]=2;
        params[20*32+:32]=-(32<<16);
        params[24*32+:32]=-800;params[25*32+:32]=600;params[26*32+:32]=1500;
        params[27*32+:32]=520;params[28*32+:32]=1017;
        params[29*32+:32]=340;params[30*32+:32]=55;
        params[7*32+:32]=30;params[8*32+:32]=600;
        params[12*32+:32]=4<<16;params[14*32+:32]=104858;
        repeat(4)@(negedge clk);rst_n=1;
        disturb_then_capture(1,0);
        disturb_then_capture(-1,0);
        disturb_then_capture(1,1);
        disturb_then_capture(-1,1);
        $display("ALL PASS recovery capture %0d",checks);$finish;
    end
endmodule
