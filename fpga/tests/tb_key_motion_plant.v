`timescale 1ns/1ps

module tb_key_motion_plant;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,tick=0,pstep=0,load=0;
    reg [52*32-1:0] params;
    reg key_neg=0,key_pos=0;
    wire [11:0] raw;
    wire signed [31:0] encoder;
    wire signed [15:0] delta;
    pend_top dut(.clk(clk),.rst_n(rst_n),.sw1_n(1'b1),.sw2_n(1'b1),
        .sw3_n(1'b1),.sw4_n(1'b1),.uart_rx_pin(1'b1),
        .adc_data_in(10'd740),.adc_otr(1'b0),.enc1_a(1'b0),.enc1_b(1'b0));
    furuta_plant plant(.clk(clk),.rst_n(rst_n),.load(load),
        .init_deg_milli(32'sd0),.init_thd_milli(32'sd0),.init_phd_milli(32'sd0),
        .step(pstep),.duty_in(dut.duty_applied),.noise_en(1'b0),
        .theta_raw(raw),.enc_pos(encoder),.enc_delta(delta));

    reg signed [31:0] enc1=0,enc2=0,measured_d=0;
    integer still_ms=0;
    wire test_dpos_good = dut.motion_ready && dut.motion_traveling ?
                          (still_ms>=40) : (delta>=-1 && delta<=1);
    always @(posedge clk) begin
        if(!rst_n) still_ms<=0;
        else if(tick) still_ms <= (delta==0) ? (still_ms<40 ? still_ms+1 : 40) : 0;
        if (!rst_n || dut.position_pid_clear) begin enc1<=encoder;enc2<=encoder;measured_d<=0;end
        else if (dut.outer_tick) begin
            enc1<=encoder;enc2<=enc1;
            measured_d <= dut.position_resume_clear ? 0 :
                -((encoder-(params[9*32+3] ? enc2 : enc1)) <<< 16) >>>
                (params[9*32+3] ? 1 : 0);
        end
    end
    task ms; begin
        @(negedge clk);tick=1;@(negedge clk);tick=0;
        repeat(252)@(negedge clk);
        pstep=1;@(negedge clk);pstep=0;@(negedge clk);
    end endtask
    integer m,fd,unused,direction,acc,dec,experiment,duration,arrived,pauses,failed;
    integer kb,kdb,kda,rope;
    integer first_ref_done,reverse_ms;
    real pos,prev,max_pos,min_pos,tilt,max_tilt,backtrack;
    reg [1023:0] csv;
    initial begin
        direction=1;acc=-1;dec=-1;experiment=0;duration=8000;csv="key_motion.csv";

        kb=-1;kdb=-1;kda=-1;rope=-1;
        unused=$value$plusargs("direction=%d",direction);
        unused=$value$plusargs("acc=%d",acc);unused=$value$plusargs("dec=%d",dec);
        unused=$value$plusargs("experiment=%d",experiment);
        unused=$value$plusargs("ms=%d",duration);unused=$value$plusargs("csv=%s",csv);
        unused=$value$plusargs("kb=%d",kb);unused=$value$plusargs("kdb=%d",kdb);
        unused=$value$plusargs("kda=%d",kda);unused=$value$plusargs("rope=%d",rope);
        repeat(4)@(negedge clk);
        params=dut.param_flat;
        params[16*32+:32]=3;params[18*32+:32]=3095;
        if(kb>=0)params[3*32+:32]=kb;
        if(kdb>=0)params[4*32+:32]=kdb;
        if(kda>=0)params[20*32+:32]=kda;
        if(rope>=0)params[51*32+:32]=rope;
        if(acc>=0)params[42*32+:32]=acc;else acc=params[42*32+:32];
        if(dec>=0)params[49*32+:32]=dec;else dec=params[49*32+:32];
        force dut.param_flat=params;force dut.ctrl_tick=tick;
        force dut.theta_raw=raw;force dut.theta_clean=raw;
        force dut.enc_pos=encoder;force dut.enc_delta=delta;
        force dut.adc_valid=1'b1;force dut.enc_fault_seen=1'b0;

        force dut.u_pwm_b.blank=1'b0;
        force dut.u_swing.state=3'd4;
        force dut.remote_key_neg=key_neg;force dut.remote_key_pos=key_pos;
        if(experiment&1) force dut.u_pid_pos.u_pid.d_err=measured_d;
        if(experiment&2) force dut.u_key_motion.dpos_good=test_dpos_good;
        rst_n=1;@(negedge clk);load=1;@(negedge clk);load=0;
        repeat(600)ms;
        if(!dut.motion_ready)$fatal(1,"plant did not become ready");
        @(negedge clk);key_neg=(direction<0);key_pos=(direction>0);
        @(negedge clk);key_neg=0;key_pos=0;
        fd=$fopen(csv,"w");
        $fdisplay(fd,"ms,position,ref,tilt,duty,p_term,d_term,integral,travel,recover,paused,rebase");
        arrived=-1;first_ref_done=-1;pauses=0;failed=0;reverse_ms=0;
        max_pos=0;min_pos=0;prev=0;max_tilt=0;backtrack=0;
        for(m=0;m<duration;m=m+1)begin
            ms;
            pos=-plant.ph*1040.0/6.283185307179586*direction;
            tilt=plant.th*180.0/3.141592653589793;
            if(tilt<0)tilt=-tilt;
            if(tilt>max_tilt)max_tilt=tilt;
            if(pos>max_pos)max_pos=pos;
            if(pos<min_pos)min_pos=pos;
            if(m>100 && pos<prev-0.001)begin backtrack=backtrack+prev-pos;reverse_ms=reverse_ms+1;end
            prev=pos;
            if(dut.motion_paused)pauses=pauses+1;
            if(dut.safe || dut.position_fault)failed=failed+1;
            if(first_ref_done<0 && dut.motion_target_ref==dut.motion_target_cmd)first_ref_done=m;
            if(arrived<0 && dut.u_position.origin!=0)arrived=m;
            $fdisplay(fd,"%0d,%f,%f,%f,%0d,%f,%f,%f,%b,%b,%b,%b",m,pos,
                $itor($signed(dut.motion_target_ref))/65536.0,tilt,$signed(dut.duty_applied),
                $itor($signed(dut.u_pid_pos.u_pid.pp_s))/65536.0,
                $itor($signed(dut.u_pid_pos.u_pid.dd_used))/65536.0,
                $itor($signed(dut.u_pid_pos.u_pid.integ))/65536.0,
                dut.motion_traveling,dut.motion_recovering,dut.motion_paused,dut.arrival_rebase);
        end
        $fclose(fd);
        $display("RESULT dir=%0d acc=%0d dec=%0d experiment=%0d ref_done=%0d arrival=%0d pos=%f overshoot=%f backtrack=%f reverse_ms=%0d tilt=%f pauses=%0d safe_ms=%0d",
            direction,acc,dec,experiment,first_ref_done,arrived,pos,max_pos-260.0,backtrack,reverse_ms,max_tilt,pauses,failed);
        if(failed || arrived<0)$fatal(1,"plant scenario failed");
        $display("ALL PASS key motion plant");$finish;
    end
endmodule
