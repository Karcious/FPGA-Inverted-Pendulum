`timescale 1ns/1ps
module tb_start_button;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0,tick=0,key=0,wr=0;
    wire press;
    reg [31:0] data=0;

    wire [52*32-1:0] flat;
    wire [31:0] mode;
    integer pulses=0,checks=0,k;
    start_button dut(.clk(clk),.rst_n(rst_n),.tick(tick),.sw1_n(key),.press(press),.long_press());
    param_regs #(.BUTTON_ENABLE(1)) regs(.clk(clk),.rst_n(rst_n),.button_press(press),
        .control_status(32'd0),.wr_en(wr),.wr_addr(6'd16),.wr_data(data),
        .rd_addr(6'd16),.rd_data(mode),.param_flat(flat));
    always @(posedge clk) if(press) pulses=pulses+1;
    task ms;begin
        repeat(4)@(negedge clk);tick=1;@(negedge clk);tick=0;
        repeat(4)@(negedge clk);
    end endtask
    task check; input ok; begin checks=checks+1;if(ok!==1)$fatal(1,"button check %0d",checks);end endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        repeat(40)ms;check(pulses==0 && mode==0);

        check(flat[3*32+:32]==32'h00048000 && flat[4*32+:32]==120*65536);

        check(flat[5*32+:32]==32'hFFFECCCD && $signed(flat[6*32+:32])==-983);
        check(flat[10*32+:32]==130 && flat[15*32+:32]==135);

        check(flat[9*32+:32]==32'd271);

        check(flat[18*32+:32]==3075 && $signed(flat[20*32+:32])==-40*65536);

        check(flat[7*32+:32]==32'd30 && flat[8*32+:32]==32'd600);

        check(flat[27*32+:32]==32'd520 && flat[29*32+:32]==32'd350
              && flat[30*32+:32]==32'd55 && flat[28*32+:32]==32'd1017);

        check(flat[32*32+:32]==32'd0);

        check(flat[33*32+:32]==32'hFFFECCCD && $signed(flat[34*32+:32])==-40*65536);
        check(flat[35*32+:32]==32'hFFFECCCD && $signed(flat[36*32+:32])==-40*65536);
        check(flat[37*32+:32]==32'd0 && flat[38*32+:32]==32'd0);

        check(flat[39*32+:32]==32'h0002_0000);
        key=1;repeat(21)ms;
        for(k=0;k<5;k=k+1)begin key=0;ms;key=1;ms;end
        check(pulses==0 && mode==0);
        key=0;repeat(19)ms;check(pulses==0);
        ms;check(pulses==1 && mode==3);
        repeat(60)ms;check(pulses==1 && mode==3);
        key=1;repeat(20)ms;key=0;repeat(20)ms;
        check(pulses==2 && mode==0);
        key=1;repeat(20)ms;
        wr=1;data=0;key=0;repeat(20)ms;wr=0;
        check(pulses==3 && mode==0);
        key=1;repeat(20)ms;key=0;repeat(20)ms;check(mode==3);
        wr=1;data=0;ms;wr=0;check(mode==0);
        rst_n=0;ms;rst_n=1;repeat(40)ms;check(mode==0);
        $display("ALL PASS button %0d",checks);$finish;
    end
endmodule
