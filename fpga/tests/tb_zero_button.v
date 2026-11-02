`timescale 1ns/1ps
module tb_zero_button;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,tick=0,key=0;
    wire short_press,long_press;
    integer shorts=0,longs=0,checks=0;
    start_button #(.LONG_PRESS_MS(2000)) dut(
        .clk(clk),.rst_n(rst_n),.tick(tick),.sw1_n(key),
        .press(short_press),.long_press(long_press));
    always @(posedge clk) begin
        if(short_press) shorts=shorts+1;
        if(long_press) longs=longs+1;
    end
    task ms;begin
        repeat(4)@(negedge clk);tick=1;@(negedge clk);tick=0;
        repeat(4)@(negedge clk);
    end endtask
    task check;input ok;begin
        checks=checks+1;if(ok!==1)$fatal(1,"zero key %0d shorts=%0d longs=%0d",checks,shorts,longs);
    end endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        repeat(2050)ms;check(shorts==0&&longs==0);
        key=1;repeat(20)ms;
        key=0;repeat(19)ms;key=1;repeat(20)ms;check(shorts==0&&longs==0);
        key=0;repeat(20)ms;repeat(100)ms;check(shorts==0&&longs==0);
        key=1;repeat(20)ms;check(shorts==1&&longs==0);

        key=0;repeat(20)ms;repeat(1999)ms;key=1;repeat(20)ms;
        check(shorts==2&&longs==0);
        key=0;repeat(20)ms;repeat(1999)ms;check(shorts==2&&longs==0);
        ms;check(shorts==2&&longs==1);
        repeat(2050)ms;check(shorts==2&&longs==1);
        key=1;repeat(20)ms;check(shorts==2&&longs==1);
        key=0;repeat(20)ms;repeat(5)ms;key=1;repeat(20)ms;check(shorts==3&&longs==1);
        $display("ALL PASS zero_button checks=%0d",checks);$finish;
    end
endmodule
