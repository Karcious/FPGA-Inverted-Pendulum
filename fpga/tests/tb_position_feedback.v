`timescale 1ns/1ps

module tb_position_feedback;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, active=0, session=0;
    reg signed [31:0] encoder=0, target=0;
    wire signed [15:0] pos;
    wire signed [31:0] error;
    wire fault;
    integer checks=0;
    reg re_zero=0;

    position_feedback dut(.clk(clk),.rst_n(rst_n),.active(active),.session(session),
        .re_zero(re_zero),.encoder(encoder),
        .target(target),.relative_pos(pos),.error_q16(error),.fault(fault),
        .leg_active(1'b0),.leg_pos());

    reg signed [31:0] fixed_encoder=10000,fixed_target=0;
    wire signed [15:0] fixed_pos;
    wire signed [31:0] fixed_error;
    wire fixed_fault;
    position_feedback #(.LATCH_ON_ENTRY(0)) fixed_zero(
        .clk(clk),.rst_n(rst_n),.active(active),.session(session),.encoder(fixed_encoder),
        .target(fixed_target),.relative_pos(fixed_pos),.error_q16(fixed_error),
        .fault(fixed_fault),.re_zero(re_zero),
        .leg_active(1'b0),.leg_pos());
    task settle; begin repeat(4) @(negedge clk); end endtask
    task check;
        input ok;
        begin checks=checks+1; if (ok !== 1'b1) $fatal(1,"position check %0d",checks); end
    endtask

    task move;
        input integer n;
        integer i;
        begin
            if (n >= 0) for (i=0;i<n;i=i+1)  begin encoder = encoder + 1; @(negedge clk); end
            else        for (i=0;i<-n;i=i+1) begin encoder = encoder - 1; @(negedge clk); end
            repeat(10) @(negedge clk);
        end
    endtask
    initial begin
        settle; encoder=10000; settle;
        rst_n=1; settle;

        session=1; settle;

        active=1; settle;
        move(4);  check(pos==4 && error==-(4<<16) && !fault);
        move(-4); check(pos==0 && error==0);

        active=0; move(96); settle;
        active=1; settle;
        check(pos==96 && error==-(96<<16));
        move(-96); check(pos==0);

        session=0; settle;
        check(pos==0);

        active=0; move(150); settle;
        session=1; settle;
        active=1; settle;
        check(pos==0 && error==0);
        move(20); check(pos==20);
        move(-20); check(pos==0);

        move(2*1040+30);  check(pos==30 && error==-(30<<16));
        move(-(2*1040+30)); check(pos==0 && error==0);

        move(5*1040);     check(pos==0 && error==0);

        move(530);        check(pos==520 && error==(510<<16));

        move(55);         check(pos==-455 && error==(455<<16));

        move(-55);        check(pos==-510 && error==(510<<16));

        move(510);        check(pos==0 && error==0);

        target=32'h7fffffff; settle; check(fault && error>0);
        active=0; settle; check(!fault);

        active=0;fixed_encoder=10300;settle;
        check(fixed_pos==300 && fixed_error==0);
        active=1;settle;check(fixed_pos==300 && fixed_error==-(300<<16));
        fixed_target=300<<16;settle;check(fixed_error==0);
        active=0;settle;fixed_encoder=10290;active=1;settle;
        check(fixed_pos==290 && fixed_error==(10<<16));
        re_zero=1;fixed_target=0;settle;re_zero=0;settle;
        check(fixed_pos==0 && fixed_error==0 && fixed_zero.origin==10290);
        active=0;fixed_encoder=10295;settle;active=1;settle;
        check(fixed_pos==5 && fixed_error==-(5<<16));

        fixed_encoder=2000000000;settle;check(fixed_pos==520 && !fixed_fault);
        re_zero=1;settle;re_zero=0;settle;
        check(fixed_pos==0 && fixed_zero.origin==2000000000 && !fixed_fault);
        active=0;settle;check(!fixed_fault);
        fixed_encoder=10000;rst_n=0;settle;rst_n=1;settle;
        check(fixed_pos==0 && fixed_zero.origin==10000);

        session=0; settle;
        session=1; settle;
        active=0; move(37); settle;
        active=1; settle;
        check(pos==0);
        move(25); check(pos==25);
        re_zero=1; settle; re_zero=0; settle;
        check(pos==0 && dut.origin==encoder);
        active=0; settle; active=1; settle;
        check(pos==0);
        move(25); check(pos==25);
        move(-25); check(pos==0);
        $display("ALL PASS position %0d",checks); $finish;
    end
endmodule
