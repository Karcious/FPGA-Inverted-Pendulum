`timescale 1ns/1ps
module tb_motion_profiles;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,tick=0,active=1,balance_confirmed=0,re_zero=0,protect=0;
    reg start_a=0,start_b=0,key_neg=0,key_zero=0,key_pos=0;
    reg signed [15:0] angle_error=0,actual_pos=0,dpos=0;
    wire ready,reached,at_target,moving,motion_paused,homing,home_fault;
    wire request,hold_motion,busy,paused,done,profile_fault,profile_id;
    wire [2:0] segment;
    wire signed [31:0] target_ref,target_cmd,command_target,command_slew;
    reg direct_request=0;
    reg signed [31:0] direct_target=0,direct_slew=0;
    integer checks=0,commands=0,ticks_run=0;
    reg signed [31:0] previous_ref;
    reg [15:0] saved_dwell;

    key_motion_target #(.READY_TICKS(3),.REACHED_TICKS(4)) motion(
        .clk(clk),.rst_n(rst_n),.tick(tick),.active(active),
        .balance_confirmed(balance_confirmed),.re_zero(re_zero),.fault(protect),
        .key_neg(key_neg),.key_zero(key_zero),.key_pos(key_pos),
        .remote_move_req(1'b0),.remote_move_counts(16'sd0),
        .profile_request(request || direct_request),
        .profile_target(direct_request ? direct_target : command_target),
        .profile_slew(direct_request ? direct_slew : command_slew),
        .hold_motion(hold_motion),.angle_error(angle_error),.actual_pos(actual_pos),.dpos(dpos),

        .slew_sel(32'sd0),.prof_slew_sel(32'sd0),.accel_sel(32'sd0),.decel_sel(32'sd0),
        .home_slew_sel(32'sd0),.home_leg(),
        .target_ref(target_ref),.target_cmd(target_cmd),.ready(ready),
        .moving(moving),.paused(motion_paused),.reached(reached),
        .homing(homing),.home_fault(home_fault),.at_target(at_target));
    motion_profile_sequencer seq(
        .clk(clk),.rst_n(rst_n),.tick(tick),
        .enable(ready && active && balance_confirmed && !protect && !home_fault),
        .cancel(key_neg || key_zero || key_pos),.re_zero(re_zero),
        .start_a(start_a),.start_b(start_b),.reached(reached),.at_target(at_target),
        .command_valid(request),.command_target(command_target),.command_slew(command_slew),
        .hold_motion(hold_motion),.busy(busy),.paused(paused),.done(done),
        .fault(profile_fault),.profile_id(profile_id),.segment(segment));
    always @(posedge clk) if(request) commands=commands+1;
    task check; input ok;
        begin checks=checks+1;
            if(ok!==1)$fatal(1,"profile check=%0d seg=%0d state=%0d ref=%0d cmd=%0d ready=%b fault=%b",
                checks,segment,seq.state,target_ref,target_cmd,ready,profile_fault);
        end
    endtask
    task ms; begin
        @(negedge clk);tick=1;@(negedge clk);tick=0;
        repeat(2)@(negedge clk);#1;
    end endtask
    task start; input a,b; begin
        @(negedge clk);start_a=a;start_b=b;
        @(negedge clk);start_a=0;start_b=0;
        repeat(3)@(negedge clk);#1;
    end endtask
    task zero_key; begin
        @(negedge clk);key_zero=1;tick=1;
        @(negedge clk);key_zero=0;tick=0;
        repeat(3)@(negedge clk);#1;
    end endtask
    task follow_one; begin
        previous_ref=target_ref;actual_pos=target_ref[31:16];ms;
        check(target_ref-previous_ref<=motion.current_slew &&
              previous_ref-target_ref<=motion.current_slew);
        check(!profile_fault && !home_fault);
    end endtask
    task finish_profile; input integer maximum_ticks;
        begin
            ticks_run=0;
            while(busy) begin
                follow_one;ticks_run=ticks_run+1;
                if(ticks_run>maximum_ticks)$fatal(1,"profile did not finish");
            end
            check(done && target_ref==0 && target_cmd==0 && reached);
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        start(1,0);check(!busy && commands==0);
        balance_confirmed=1;repeat(15)ms;check(ready && reached);
        start(1,1);check(!busy && commands==0);
        start(1,0);check(busy && !profile_id && segment==0 && commands==1);
        check(target_cmd==-(130<<16) && motion.current_slew==1136);
        check(target_ref==0 && !reached);
        start(0,1);check(!profile_id && segment==0 && commands==1);
        angle_error=240;previous_ref=target_ref;repeat(10)ms;
        check(motion_paused && target_ref==previous_ref && segment==0);
        angle_error=0;actual_pos=-17;ms;
        check(motion_paused && target_ref==previous_ref);
        actual_pos=0;
        while(seq.state!=3)follow_one;
        check(reached && at_target && segment==0 && commands==1);

        repeat(100)follow_one;saved_dwell=seq.dwell_count;
        actual_pos=target_ref[31:16]+3;ms;
        check(paused && seq.dwell_count==saved_dwell);
        angle_error=61;repeat(10)ms;check(seq.dwell_count==saved_dwell);
        angle_error=0;dpos=2;actual_pos=target_ref[31:16];ms;
        check(paused && seq.dwell_count==saved_dwell);
        dpos=0;repeat(4)ms;check(seq.dwell_count==saved_dwell);
        finish_profile(26000);check(commands==3 && segment==2);
        check(seq.command_slew==1136);

        commands=0;start(0,1);
        check(busy && profile_id && !done && !reached && segment==0);
        check(target_cmd==(260<<16) && motion.current_slew==1515 && commands==1);
        while(segment==0)follow_one;
        check(target_cmd==5679787 && motion.current_slew==757 && commands==2);
        while(segment==1)follow_one;
        check(target_cmd==-5679787 && motion.current_slew==1515 && commands==3);
        finish_profile(22000);check(commands==4 && segment==3);

        @(negedge clk);start_a=1;@(negedge clk);start_a=0;key_zero=1;tick=1;
        @(negedge clk);key_zero=0;tick=0;repeat(3)@(negedge clk);#1;
        check(!busy && !done && target_cmd==0 && motion.current_slew==8192);
        start(1,0);repeat(20)follow_one;
        @(negedge clk);key_pos=1;@(negedge clk);key_pos=0;
        repeat(3)@(negedge clk);#1;
        check(!busy && target_cmd==(260<<16) && motion.current_slew==8192);
        zero_key;
        while(moving)follow_one;repeat(5)follow_one;

        start(0,1);repeat(20)follow_one;
        @(negedge clk);re_zero=1;actual_pos=0;
        @(negedge clk);re_zero=0;repeat(3)@(negedge clk);#1;
        check(!busy && !ready && target_ref==0 && target_cmd==0);
        repeat(15)ms;check(ready && !busy && !done);

        start(1,0);angle_error=240;previous_ref=target_ref;
        repeat(59999)ms;check(!profile_fault && busy && seq.elapsed_ms==59999);
        ms;check(profile_fault && !busy && !moving && target_ref==previous_ref);
        check(target_cmd==target_ref);start(0,1);check(profile_fault && !busy);
        zero_key;angle_error=0;repeat(5)ms;check(!profile_fault);

        force seq.command_target=(521<<16);
        start(1,0);check(profile_fault && !busy && target_cmd==target_ref);
        release seq.command_target;zero_key;repeat(5)ms;

        @(negedge clk);direct_request=1;direct_target=32'sh7fffffff;direct_slew=32'sh7fffffff;
        @(negedge clk);direct_request=0;#1;
        check(target_cmd==(520<<16) && motion.current_slew==3787);
        @(negedge clk);direct_request=1;direct_target=32'sh80000000;direct_slew=-1;
        @(negedge clk);direct_request=0;#1;
        check(target_cmd==-(520<<16) && motion.current_slew==1);
        zero_key;repeat(5)ms;

        start(1,0);protect=1;ms;check(!busy && !ready && !done);
        protect=0;actual_pos=0;repeat(15)ms;check(ready && !busy);
        start(0,1);active=0;ms;check(!busy && !ready && !profile_fault);
        $display("ALL PASS motion_profiles checks=%0d",checks);$finish;
    end
endmodule
