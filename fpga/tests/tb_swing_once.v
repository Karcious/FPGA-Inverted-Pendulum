`timescale 1ns/1ps
module tb_swing_once;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,tick=0,request=0,safe=0,valid=1,gains=1;
    reg [15:0] theta=1017;
    reg [11:0] raw=2960;
    reg fresh=1;
    reg signed [31:0] pos=50000,step_target=341,kp=4*65536,kd=104858;
    reg signed [15:0] delta=0;
    reg [31:0] switch_deg=15,switch_dps=120,timeout_ms=1500,capture_ms=10;

    reg signed [31:0] brake_gain=0;

    reg signed [31:0] settle_gain=0;

    reg mode_handoff=0;
    wire balance;
    wire signed [15:0] drive;
    wire [2:0] state;
    wire [3:0] reason;
    integer k,checks=0;
    swing_once dut(.clk(clk),.rst_n(rst_n),.tick(tick),.request(request),
        .balance_safe(safe),.sensor_valid(valid),.gains_valid(gains),
        .raw_angle(raw),.sensor_fresh(fresh),.theta(theta),.center(16'd3095),.hang(16'd1017),.position(pos),.delta(delta),
        .pulse_duty(-32'sd600),.pulse_ms(32'd600),.timeout_ms(timeout_ms),.travel_max(32'd450),
        .switch_deg(switch_deg),.switch_dps(switch_dps),.step_target(step_target),
        .step_kp(kp),.step_kd(kd),.capture_ms(capture_ms),.brake_gain(brake_gain),
        .settle_gain(settle_gain),.mode_handoff(mode_handoff),
        .entered(),.confirmed(),.gap_seen(),.gap_active(),.telemetry_pos(),.fault_code(reason),.balance(balance),.drive(drive),.state(state));
    task step;begin repeat(14)@(negedge clk);tick=1;@(negedge clk);tick=0;repeat(14)@(negedge clk);end endtask
    task check;input ok;begin checks=checks+1;if(ok!==1)$fatal(1,"step swing %0d state=%0d duty=%0d",checks,state,drive);end endtask
    task start;begin
        request=0;step;theta=1017;delta=0;pos=50000;safe=0;valid=1;gains=1;
        step_target=341;kp=4*65536;kd=104858;capture_ms=10;timeout_ms=1500;
        brake_gain=0;
        settle_gain=0;
        mode_handoff=0;
        raw=2960;fresh=1;request=1;repeat(140)step;
        check(state==2 && drive==-600 && !balance);
    end endtask

    task wait_fault;
        input integer limit;
        begin k=0; while(state!=5 && k<limit) begin step;k=k+1;end end
    endtask

    task sweep_raw;
        input [11:0] target;
        integer d;
        begin
            while(raw!=target) begin
                d = raw>target ? raw-target : target-raw;
                if(d>150) d=150;
                raw = raw>target ? raw-d : raw+d;
                step;
            end
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;step;check(drive==0);
        start;

        check(dut.origin==50000);
        pos=50320;repeat(5)step;check(drive>0 && drive<=600);
        repeat(10)step;check(drive==-84);
        pos=50341;repeat(15)step;check(drive==0);
        pos=50360;repeat(15)step;check(drive==76);
        request=0;#1;check(drive==0);step;check(state==0);
        start;repeat(620)step;check(state==3 && drive==0);

        request=0;step;
        check(state==0);
        sweep_raw(912);
        theta=3095;
        delta=0;mode_handoff=1;request=1;
        step;check(state==1);
        repeat(24)step;
        check(state==6);
        check(balance==1);
        repeat(20)step;mode_handoff=0;
        check(state==4 && balance==1);
        check(dut.confirmed==1);

        request=0;step;mode_handoff=0;request=1;repeat(40)step;
        check(state==1);

        start;repeat(140)step;check(state==2 && !balance);

        start; brake_gain=32'sd1<<16;
        k=0; while (state!=3 && k<2000) begin step;k=k+1; end
        check(state==3 && drive==0);
        theta=1117; step;
        check(drive==100);
        theta=917; step; check(drive==-100);
        theta=1017+800; step; check(drive==600);

        delta=4; step; check(state==5 && reason==3 && drive==0);

        delta=0; step; check(state!=5 && dut.retry_count==1);

        start; brake_gain=32'sd1<<16;
        k=0; while (state!=3 && k<2000) begin step;k=k+1; end
        for (k=0;k<13;k=k+1) begin theta=theta+150; step; end
        check(drive==600);
        theta=theta+150; step; check(drive==-600);
        start;repeat(620)step;check(state==3 && drive==0);

        theta=2017;step;theta=3017;step;check(state!=4);
        theta=3095;repeat(35)step;check(balance && drive==0);
        safe=1;step;check(state==5 && reason==8 && !balance);

        safe=0;step;check(state!=5 && dut.retry_count==1);

        start;pos=50451;step;check(state==5 && reason==5 && drive==0);
        step;check(state!=5 && dut.retry_count==1);
        start;delta=4;step;check(state==5 && reason==3);

        delta=0;step;check(state!=5 && dut.retry_count==1);

        start;delta=4;step;check(state==5 && reason==3);
        step;check(state==0 && dut.retry_count==1);
        step;check(state==1 && dut.retry_count==1);
        repeat(6)step;check(state==1 && dut.retry_count==1);
        delta=0;

        pos=50451;repeat(3)step;check(state==1 && dut.retry_count==1);
        pos=50000;
        start;valid=0;#1;check(drive==0);step;check(reason==1 && state==5);

        start;theta=4167;step;check(state!=5 && drive!=0);

        theta=2017;step;check(state!=5);
        theta=4167;step;check(state!=5);
        theta=2017;step;check(state==5 && reason==4);

        start;
        theta=4167;step;theta=4167;step;
        theta=1017;step;theta=1017;step;
        theta=4167;step;theta=4167;step;
        check(state!=5);
        start;timeout_ms=700;wait_fault(4000);check(state==5 && reason==7 && dut.retry_count==0);

        start;timeout_ms=700;step_target=340;

        step;
        check(dut.swing_target==340);
        wait_fault(4000);check(state==5 && reason==7 && dut.retry_count==0);
        step;check(state!=5 && dut.retry_count==1 && dut.swing_target==360);
        wait_fault(4000);check(state==5 && reason==7 && dut.retry_count==1);
        step;check(dut.retry_count==2 && dut.swing_target==380);
        wait_fault(4000);check(state==5 && reason==7 && dut.retry_count==2);
        step;check(dut.retry_count==3 && dut.swing_target==400);
        wait_fault(4000);check(state==5 && reason==7 && dut.retry_count==3);
        step;check(state==5 && dut.retry_count==3);

        start;timeout_ms=700;step_target=390;
        step;step;
        check(dut.swing_target==390);
        wait_fault(4000);step;check(dut.retry_count==1 && dut.swing_target==400);
        wait_fault(4000);step;check(dut.retry_count==2 && dut.swing_target==400);

        start;timeout_ms=700;step_target=340;
        wait_fault(4000);step;check(dut.retry_count==1 && dut.swing_target==360);

        wait_fault(4000);step;check(dut.retry_count==2 && dut.swing_target==380);
        wait_fault(4000);step;check(dut.retry_count==3 && dut.swing_target==400);
        timeout_ms=5000;
        k=0; while (state!=3 && k<1200) begin step;k=k+1; end
        check(state==3 && drive==0);
        theta=2017;step;theta=3017;step;
        theta=3095;repeat(60)step;
        check(state==4 && dut.confirmed);
        check(dut.retry_count==0 && dut.swing_target==340);

        delta=4;step;check(state==5 && reason==3);
        delta=0;step;check(dut.retry_count==1 && dut.swing_target==340);
        start;kp=-1;#1;check(drive==0);step;check(reason==2 && state==5);
        start;step_target=420;step;check(state==5 && reason==2);
        start;capture_ms=0;step;check(state==5 && reason==2);
        start;request=0;step;step_target=-341;request=1;repeat(140)step;
        check(state==2 && drive==600);

        start;request=0;step;theta=1300;request=1;
        k=0;
        while(dut.rest_count<dut.REST_MAX && state!=5 && k<30000) begin step;k=k+1;end
        check(state!=5 && dut.rest_count==dut.REST_MAX && dut.retry_count==0);
        wait_fault(4000);
        check(state==5 && reason==6 && dut.retry_count==0);
        step;check(state==5 && dut.retry_count==0);

        start;capture_ms=40;
        theta=2017;step;theta=3017;step;theta=3095;repeat(45)step;check(state!=4);
        theta=2800;step;theta=3095;repeat(45)step;check(state!=4);
        repeat(20)step;check(balance);

        start;switch_dps=450;
        theta=2017;step;theta=2900;repeat(25)step;
        theta=2930;step;check(state==6 && balance && dut.entered && !dut.confirmed);
        safe=1;step;check(reason==8 && dut.entered && !balance);
        start;switch_dps=450;theta=2017;step;theta=3017;step;theta=3095;repeat(35)step;
        check(state==4 && dut.confirmed);

        start;switch_dps=450;capture_ms=100;
        theta=2017;step;theta=2900;repeat(25)step;theta=2930;step;
        for(k=0;k<510;k=k+1) begin theta=(k%40<20)?16'd3095:16'd3100;step;end

        check(state==4 && dut.confirmed);
        start;switch_dps=450;capture_ms=100;
        theta=2017;step;theta=2900;repeat(25)step;theta=2930;step;

        for(k=0;k<510 && state!=5;k=k+1) begin theta=(k%40<20)?16'd2900:16'd3300;step;end
        check(state==5 && reason==10 && dut.entered && !dut.confirmed && dut.retry_count==0);

        start;sweep_raw(50);raw=0;valid=0;step;
        check(state==3 && dut.gap_active && dut.gap_seen && drive==0);
        repeat(16)step;raw=4030;valid=1;step;raw=3900;step;
        check(state==3 && !dut.gap_active && dut.valid_count==0);

        start;sweep_raw(50);raw=0;valid=0;wait_fault(200);
        check(state==5 && reason==9 && drive==0);
        step;check(state!=5 && dut.retry_count==1);
        start;raw=50;step;raw=0;valid=0;fresh=0;step;
        check(state==5 && reason==1);

        raw=2960;start;raw=0;valid=0;step;check(state==5 && reason==1);
        start;pos=50042;repeat(3)step;check(dut.telemetry_pos==42);

        request=0;step;request=1;step;
        settle_gain=-32'sd2*65536;
        repeat(25)step;
        check(state==1 && dut.valid_count==20 && dut.settle_zone);
        check(drive==0);
        theta=1200;step;
        check(drive==-366);
        repeat(20)step;check(drive==0);

        request=0;step;request=1;step;
        check(state==1 && dut.valid_count<20);
        theta=1017;step;check(drive==0);
        repeat(25)step;
        theta=1367;step;
        check(drive==-600);
        repeat(20)step;

        theta=2100;step;check(drive==0);
        repeat(20)step;
        settle_gain=0;
        request=0;

        settle_gain=-32'sd2*65536;
        request=0;step;request=1;step;
        theta=1017;raw=2960;valid=1;fresh=1;
        repeat(25)step;
        check(state==1 && dut.valid_count==20 && dut.settle_zone);
        theta=1200;step;check(drive==-366);
        theta=1350;raw=0;valid=0;step;
        check(drive==0);
        repeat(60)step;
        check(state==1 && reason==0 && drive==0);

        theta=3000;raw=2960;valid=1;step;
        check(state==1 && reason==0);
        repeat(25)step;
        check(state==1 && reason==0 && dut.valid_count==20);
        settle_gain=0;

        start;sweep_raw(4021);
        check(state!=5 && reason==0);
        raw=426;step;
        check(state!=5 && reason==0);
        check(dut.previous_raw==4021);
        raw=0;valid=0;step;
        check(state!=5 && reason==0);
        check(dut.gap_active && dut.gap_seen);

        check(dut.previous_raw==0);

        raw=1681;valid=1;step;
        check(state!=5 && reason==0 && dut.previous_raw==0);
        raw=4030;step;
        check(state!=5 && reason==0 && dut.previous_raw==4030);
        for(k=5;k<=30;k=k+1) begin
            switch_deg=k;repeat(3)@(negedge clk);
            check(dut.capture_angle==k*2078/180);
        end
        for(k=10;k<=600;k=k+1) begin
            switch_dps=k;repeat(3)@(negedge clk);
            check(dut.capture_velocity==k*2078/9000);
        end
        switch_deg=32'hffffffff;switch_dps=32'hffffffff;
        repeat(3)@(negedge clk);
        check(!dut.config_ok && dut.capture_angle==0 && dut.capture_velocity==0);
        $display("ALL PASS swing %0d",checks);$finish;
    end
endmodule
