`timescale 1ns/1ps
module tb_param_status;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,write_en=0;
    reg [5:0] address=0;
    reg [31:0] data=0,status=0;
    wire [31:0] result;

    wire [52*32-1:0] flat;
    integer checks=0;
    param_regs #(.STATUS_ENABLE(1)) dut(.clk(clk),.rst_n(rst_n),.control_status(status),
        .wr_en(write_en),.wr_addr(address),.wr_data(data),.rd_addr(address),.rd_data(result),.param_flat(flat));
    task settle;begin repeat(3)@(negedge clk);end endtask
    task check;input ok;begin checks=checks+1;if(ok!==1)$fatal(1,"status %0d",checks);end endtask
    initial begin
        settle;rst_n=1;address=31;status=32'h00000125;settle;
        check(result==status && flat[31*32+:32]==status);
        write_en=1;data=32'hffffffff;settle;check(result==status);
        address=63;settle;check(result==32'hdeadbeef);

        write_en=0;address=24;settle;check($signed(result)==-900);
        address=16;settle;check(result==0);
        address=12;settle;check(result==4*65536);
        address=14;settle;check(result==104858);
        address=25;settle;check(result==600);
        address=26;settle;check(result==1500);

        address=29;settle;check(result==350);

        address=30;settle;check(result==55);

        address=32;settle;check(result==32'd0);

        address=33;settle;check($signed(result)==-78643);
        address=38;settle;check(result==32'd0);

        address=39;settle;check(result==32'h00020000);

        address=40;settle;check(result==32'd13107);
        address=46;settle;check(result==32'd4);

        address=47;settle;check(result==32'h00004000);

        address=48;settle;check(result==32'd0);

        address=42;settle;check(result==32'd98);
        address=49;settle;check(result==32'd30);

        address=50;settle;check(result==32'd1);

        address=51;settle;check(result==32'd0);
        address=52;settle;check(result==32'hdeadbeef);
        $display("ALL PASS status %0d",checks);$finish;
    end
endmodule
