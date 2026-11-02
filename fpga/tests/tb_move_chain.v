`timescale 1ns/1ps

module tb_move_chain;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,tick=0,pstep=0,load=0;
    reg [52*32-1:0] params;
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

    task ms; begin
        @(negedge clk);tick=1;@(negedge clk);tick=0;
        repeat(252)@(negedge clk);
        pstep=1;@(negedge clk);pstep=0;@(negedge clk);
    end endtask

    reg arr_seen, started_seen;
    always @(posedge clk) begin
        if (!rst_n) begin arr_seen<=1'b0; started_seen<=1'b0; end
        else begin
            if (dut.motion_key_arrival) arr_seen<=1'b1;
            if (dut.motion_traveling)   started_seen<=1'b1;
        end
    end

    integer k,m,fd,unused,seg_ms,total_ms;
    real pos,total_travel;
    reg [1023:0] csv;
    reg ok;
    initial begin
        csv="move_chain.csv";
        unused=$value$plusargs("csv=%s",csv);
        repeat(4)@(negedge clk);
        params=dut.param_flat;
        params[16*32+:32]=3;params[18*32+:32]=3095;
        force dut.param_flat=params;force dut.ctrl_tick=tick;
        force dut.theta_raw=raw;force dut.theta_clean=raw;
        force dut.enc_pos=encoder;force dut.enc_delta=delta;
        force dut.adc_valid=1'b1;force dut.enc_fault_seen=1'b0;
        force dut.u_pwm_b.blank=1'b0;
        force dut.u_swing.state=3'd4;
        force dut.remote_key_neg=1'b0;force dut.remote_key_pos=1'b0;
        rst_n=1;@(negedge clk);load=1;@(negedge clk);load=0;
        repeat(600)ms;
        if(!dut.motion_ready)$fatal(1,"plant did not become ready");

        fd=$fopen(csv,"w");
        $fdisplay(fd,"ms,seg,position,ref,tilt,duty,travel,ready,travel_phase");

        total_ms=0;total_travel=0;ok=1;
        for(k=0;k<4 && ok;k=k+1)begin

            m=0;
            while(!(dut.motion_traveling==0 && dut.motion_ready==1 && dut.motion_key_arrival==0 && m<3000))begin
                ms;m=m+1;total_ms=total_ms+1;
            end
            if(m>=3000)begin
                $display("SEG %0d: 等不到空闲（traveling=%b ready=%b arrival=%b）",
                         k,dut.motion_traveling,dut.motion_ready,dut.motion_key_arrival);
                ok=0;
            end else begin
                arr_seen=0;started_seen=0;
                @(negedge clk);
                force dut.remote_move_req_r=1'b1;
                force dut.remote_move_counts_r=16'sd390;
                @(negedge clk);
                force dut.remote_move_req_r=1'b0;
                force dut.remote_move_counts_r=16'sd0;
                seg_ms=0;
                while(!arr_seen && seg_ms<8000)begin
                    pos=-plant.ph*1040.0/6.283185307179586;
                    $fdisplay(fd,"%0d,%0d,%f,%f,%f,%0d,%b,%b,%b",total_ms,k,pos,
                        $itor($signed(dut.motion_target_ref))/65536.0,
                        plant.th*180.0/3.141592653589793,$signed(dut.duty_applied),
                        dut.motion_traveling,dut.motion_ready,dut.u_key_motion.travel_phase);
                    ms;seg_ms=seg_ms+1;total_ms=total_ms+1;
                end
                if(!arr_seen)begin
                    $display("SEG %0d: 8000 ms 内没到点（started=%b traveling=%b ready=%b ref=%0d cmd=%0d）",
                             k,started_seen,dut.motion_traveling,dut.motion_ready,
                             $signed(dut.motion_target_ref),$signed(dut.motion_target_cmd));
                    ok=0;
                end else if(!started_seen)begin
                    $display("SEG %0d: 命令被吃了 —— 从没开始走（ready=%b）",k,dut.motion_ready);
                    ok=0;
                end else begin
                    $display("SEG %0d: 走了 %0d ms 到点，ready=%b，地址75 误差=%0d 码，次数=%0d",
                             k,seg_ms,dut.motion_ready,
                             $signed(dut.motion_arrival_error),dut.motion_arrival_count);
                    total_travel=total_travel+pos;
                end
            end
        end
        $fclose(fd);
        $display("总位移 %.1f 码（目标 1560 码），用了 %0d ms",
                 -plant.ph*1040.0/6.283185307179586,total_ms);
        if(!ok)$fatal(1,"move chain broke");
        $display("ALL PASS move chain");
        $finish;
    end
endmodule
