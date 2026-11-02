`timescale 1ns/1ps
module tb_key_soft_edges;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,tick=0,neg=0,pos=0;
    reg signed[15:0] actual=0;
    reg signed[31:0] slew=8192,accel=10,decel=30;
    wire signed[31:0] ref_pos,cmd_pos;
    wire ready;
    integer checks=0,n,step_size,previous_step,peak,last_step;
    reg signed[31:0] previous;
    key_motion_target #(.READY_TICKS(1),.REACHED_TICKS(1)) dut(
        .clk(clk),.rst_n(rst_n),.tick(tick),.active(1'b1),.balance_confirmed(1'b1),
        .re_zero(1'b0),.fault(1'b0),.key_neg(neg),.key_zero(1'b0),.key_pos(pos),
        .remote_move_req(1'b0),.remote_move_counts(16'sd0),
        .profile_request(1'b0),.profile_target(32'sd0),.profile_slew(32'sd0),.hold_motion(1'b0),
        .angle_error(16'sd0),.actual_pos(actual),.dpos(16'sd0),
        .slew_sel(slew),.prof_slew_sel(32'sd3787),.accel_sel(accel),.decel_sel(decel),
        .home_slew_sel(32'sd0),.target_ref(ref_pos),.target_cmd(cmd_pos),.ready(ready));
    task check;input ok;input[8*96-1:0] label;begin
        checks=checks+1;if(ok!==1)$fatal(1,"soft edges: %0s n=%0d step=%0d prev=%0d ref=%0d",label,n,step_size,previous_step,ref_pos);
    end endtask
    task ms;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(4)@(negedge clk);end endtask
    task press;input integer direction;begin
        @(negedge clk);neg=direction<0;pos=direction>0;
        @(negedge clk);neg=0;pos=0;
    end endtask
    task move;input integer direction;begin
        press(direction);n=0;peak=0;previous_step=0;last_step=0;
        while(ref_pos!=cmd_pos && n<6500)begin
            previous=ref_pos;actual=ref_pos[31:16];ms;
            step_size=direction*(ref_pos-previous);
            check(step_size>=0 && step_size<=slew,"bounded forward reference without overshoot");
            if(n==0 && accel>0)check(step_size==0,"first enabled-ramp step starts at zero velocity");

            if(n==1 && accel>0)check(step_size>0 && step_size<=accel,"next step stays within configured acceleration");
            if(n>0 && accel>0 && step_size>previous_step)
                check(step_size-previous_step<=accel,"acceleration bound");
            if(n>0 && decel>0 && step_size<previous_step && ref_pos!=cmd_pos)
                check(previous_step-step_size<=30,"deceleration bound before exact endpoint snap");
            if(step_size>peak)peak=step_size;
            previous_step=step_size;last_step=step_size;n=n+1;
        end
        check(ref_pos==cmd_pos,"finite arrival at the exact command");
        check(peak==slew,"nominal cruise speed remains available");
        if(accel>0)begin check(last_step<=64,"near-zero reference speed at the endpoint");check(n>1300 && n<5000,"bounded motion duration");end
        else begin check(n==2080,"zero parameters retain the old fixed-speed timing");check(last_step==8192,"explicit off switch bypasses the smoothing");end
        $display("EDGE_RESULT dir=%0d accel=%0d decel=%0d ticks=%0d last_step=%0d peak=%0d",direction,accel,decel,n,last_step,peak);
        actual=ref_pos[31:16];repeat(41)ms;actual=0;repeat(2)ms;
        check(ref_pos==0 && cmd_pos==0 && ready,"arrival still rebases and permits the next key");
    end endtask
    initial begin
        actual=20;
        repeat(3)@(negedge clk);rst_n=1;ms;
        check(dut.homing && cmd_pos==0,"initial home starts before key-ready");
        n=0;
        while(ref_pos!=0 && n<200)begin actual=ref_pos[31:16];ms;n=n+1;end
        check(n==160 && ref_pos==0,"enabled key smoothing preserves the original initial-home speed");
        actual=0;repeat(8)ms;check(ready,"ready");
        move(1);move(-1);accel=0;decel=0;move(1);
        slew=13107;accel=98;decel=30;move(1);move(-1);
        $display("ALL PASS key soft edges %0d",checks);$finish;
    end
endmodule
