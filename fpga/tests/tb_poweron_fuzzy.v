`timescale 1ns/1ps
module tb_poweron_fuzzy;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,tick=0;
    reg [15:0] theta=0;
    reg [11:0] raw_angle=2950;
    reg signed [31:0] encoder=10000;
    reg write_en=0;
    reg [5:0] write_addr=0;
    reg [31:0] write_data=0;

    reg [31:0] saved_params[0:46];
    reg [15:0] center;
    reg [31:0] filter_base;
    reg signed [31:0] base_p,base_d,fixed_angle,fuzzy_angle;
    reg signed [31:0] outer_trace[0:299];
    reg signed [15:0] duty_trace[0:299];
    reg watch_resume=0,resume_seen=0,first_resume_clear=0;
    reg signed [31:0] saved_outer_error,first_resume_integral;
    integer checks=0,n,a,resume_guard;
    pend_top dut(.clk(clk),.rst_n(rst_n),
        .sw1_n(1'b1),.sw2_n(1'b1),.sw3_n(1'b1),.sw4_n(1'b1),
        .uart_rx_pin(1'b1),.adc_data_in(10'd740),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0));

    always @(posedge clk) begin
        if(watch_resume && !resume_seen && dut.u_pid_pos.en) begin
            resume_seen=1;
            first_resume_clear=dut.u_pid_pos.clear;
            first_resume_integral=dut.u_pid_pos.u_pid.integ;
        end
    end

    task check;input ok;input [8*96-1:0] name;begin
        checks=checks+1;
        if(ok!==1)$fatal(1,"%0s angle=%0d position=%0d duty=%0d",name,
            dut.ang_out,dut.outer_out,dut.duty_cmd);
    end endtask
    task step;begin
        @(negedge clk);tick=1;
        @(negedge clk);tick=0;repeat(80)@(negedge clk);#1;
    end endtask
    task put_param;input [5:0] addr;input [31:0] value;begin
        @(negedge clk);write_addr=addr;write_data=value;write_en=1;
        @(negedge clk);write_en=0;repeat(3)@(negedge clk);#1;
        check(dut.param_flat[addr*32+:32]===value,"protocol write reaches real register bank");
    end endtask
    task check_preserved;begin

        for(a=1;a<47;a=a+1)
            if(a!=9 && a!=16 && a!=31)
                check(dut.param_flat[a*32+:32]===saved_params[a],"user parameter remains unchanged");
        check(dut.param_flat[9*32+:8]===filter_base[7:0],"fuzzy toggle preserves all original filter bits");
    end endtask
    task start_trial;input fuzzy;begin
        theta=center;encoder=10000;
        rst_n=0;repeat(4)@(negedge clk);rst_n=1;repeat(4)step;
        put_param(9,fuzzy ? (filter_base|32'h00000100) : (filter_base&32'hfffffeff));
        put_param(16,2);repeat(100)step;
        check(!dut.pid_clear && !dut.position_pid_clear,"closed loop is running with actual defaults");
        check_preserved;
    end endtask

    initial begin
        force dut.ctrl_tick=tick;
        force dut.theta=theta;
        force dut.enc_pos=encoder;
        force dut.enc_delta=16'sd0;
        force dut.adc_valid=1'b1;
        force dut.theta_raw=raw_angle;
        force dut.enc_fault_seen=1'b0;

        force dut.u_dbg.u_proto.p_wr_en=write_en;
        force dut.u_dbg.u_proto.p_wr_addr=write_addr;
        force dut.u_dbg.u_proto.p_wr_data=write_data;
        repeat(4)@(negedge clk);rst_n=1;repeat(4)step;
        for(a=0;a<47;a=a+1)saved_params[a]=dut.param_flat[a*32+:32];
        center=saved_params[18][15:0];filter_base=saved_params[9];
        base_p=$signed(saved_params[3]);base_d=$signed(saved_params[4]);theta=center;
        raw_angle=center-saved_params[15][15:0];

        check(saved_params[0]==={8'd0,dut.u_dbg.VER,dut.u_dbg.REV},"REV power-on identity");
        check(dut.duty_cmd==0,"power-on motor command is disabled");
        $display("ACTUAL DEFAULTS Kp=%0d Kd=%0d outer=%0d/%0d/%0d center=%0d filters=%h",
            base_p,base_d,$signed(saved_params[5]),$signed(saved_params[6]),
            $signed(saved_params[20]),center,filter_base);

        start_trial(0);theta=center-12;repeat(150)step;fixed_angle=dut.ang_out;
        start_trial(1);theta=center-12;repeat(150)step;fuzzy_angle=dut.ang_out;
        check(dut.kp_ang_eff===base_p && dut.ki_ang_eff==0 && dut.kd_ang_eff===base_d,
            "stationary fuzzy inner gains return to user's base values");
        check(fuzzy_angle===fixed_angle,"stationary offset has no hidden gain reduction or increase");

        start_trial(0);
        for(n=1;n<=20;n=n+1)begin theta=center-2*n;step;end
        fixed_angle=dut.ang_out;
        start_trial(1);
        for(n=1;n<=20;n=n+1)begin theta=center-2*n;step;end
        fuzzy_angle=dut.ang_out;
        check(dut.kp_ang_eff!=base_p && dut.kd_ang_eff!=base_d,
            "bounded scheduling reaches actual inner PID");
        check(fuzzy_angle!==fixed_angle,"scheduled gains change real inner control output");
        $display("INNER MOVING fixed=%0d fuzzy=%0d Q16",fixed_angle,fuzzy_angle);

        start_trial(0);
        for(n=0;n<300;n=n+1)begin
            encoder=10000+((n*7)%97)-48;step;
            outer_trace[n]=dut.outer_out;duty_trace[n]=dut.duty_cmd;
        end
        start_trial(1);
        for(n=0;n<300;n=n+1)begin
            encoder=10000+((n*7)%97)-48;step;
            check(dut.outer_out===outer_trace[n],"fuzzy toggle preserves outer output on same trajectory");
            check(dut.duty_cmd===duty_trace[n],"quiet inner plus same outer gives identical motor command");
            check(dut.kp_pos_eff===dut.kp_arm_sel && dut.ki_pos_eff===dut.param_flat[6*32+:32] &&
                dut.kd_pos_eff===dut.kd_arm_sel,
                "outer effective gain telemetry follows real fixed inputs");
        end
        check_preserved;
        if(saved_params[6]!=0)
            check(dut.u_pid_pos.u_pid.integ!=0,"real outer integral has accumulated");
        saved_outer_error=dut.u_pid_pos.u_pid.err_d;
        put_param(16,0);repeat(40)@(negedge clk);#1;
        check(dut.position_pid_clear && !dut.u_pid_pos.en,"stop inhibits real outer samples");
        check(dut.u_pid_pos.u_pid.integ==0 && dut.u_pid_pos.u_pid.d_lpf==0,
            "stopped outer clears I and D without a sample");
        check(dut.duty_cmd==0,"stop gates motor command");

        put_param(13,-64*32'sd65536);
        repeat(40)@(negedge clk);
        check(!dut.u_pid_pos.en && dut.position_pid_clear,
            "target change occurs while stopped outer samples are inhibited");
        watch_resume=1;resume_seen=0;
        put_param(16,2);
        resume_guard=0;
        while(!resume_seen && resume_guard<40)begin step;resume_guard=resume_guard+1;end
        watch_resume=0;
        check(resume_seen,"resumed outer loop accepts a fresh sample");
        check(dut.u_pid_pos.u_pid.err_d!==saved_outer_error,
            "resume stimulus actually changes sampled error history");
        $display("OUTER RESUME old_error=%0d new_error=%0d first_clear=%b D=%0d I=%0d",
            saved_outer_error,dut.u_pid_pos.u_pid.err_d,first_resume_clear,
            dut.u_pid_pos.u_pid.d_lpf,dut.u_pid_pos.u_pid.integ);
        check(dut.u_pid_pos.u_pid.d_lpf==0,
            "first resumed outer sample has no derivative kick from stale history");
        check(first_resume_integral==0 && first_resume_clear,
            "first resumed outer sample starts with cleared I and explicitly seeds D history");

        put_param(16,0);put_param(13,saved_params[13]);
        check_preserved;
        $display("ALL PASS poweron/conservative fuzzy checks=%0d",checks);$finish;
    end
    initial begin #10000000;$fatal(1,"timeout");end
endmodule
