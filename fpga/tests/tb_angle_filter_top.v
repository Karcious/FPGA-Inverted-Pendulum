`timescale 1ns/1ps
module tb_angle_filter_top;
reg clk=0;always #10 clk=~clk;
reg rst_n=0,key=1;wire pwm,an1,an2,pwm_b,bn1,bn2,adc_clk,adc_oe,tx;wire[3:0]led;
reg[11:0]raw=2960;reg valid=0;
    pend_top dut(.sw1_n(key),.sw2_n(1'b1),.sw3_n(1'b1),.sw4_n(1'b1),
        .clk(clk),.rst_n(rst_n),.uart_rx_pin(1'b1),.uart_tx_pin(tx),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe),.adc_data_in(10'd740),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.pwm(pwm),.an1(an1),.an2(an2),
        .pwm_b(pwm_b),.bn1(bn1),.bn2(bn2),.led(led));

task put(input [11:0] x);begin
@(negedge clk);raw=x;valid=1;@(negedge clk);valid=0;repeat(6)@(negedge clk);
end endtask
initial begin
force dut.theta_raw=raw;force dut.adc_valid=valid;
repeat(4)@(negedge clk);rst_n=1;
repeat(5)put(2960);
if(dut.theta!==3095 || dut.ch_theta!==3095)$fatal(1,"trim/control/telemetry mismatch");
put(3460);put(2960);put(2960);
if(dut.theta!==3095)$fatal(1,"spike reached controller");
put(0);
if(dut.sensor_ok!==0)$fatal(1,"filter masked raw range fault");
put(2960);put(2960);
if(dut.theta!==3095)$fatal(1,"raw endpoint impulse not rejected");
if(dut.u_dbg.u_proto.u_params.MAGIC!==32'h00020719)$fatal(1,"wrong revision");
$display("ALL PASS angle filter top: control/telemetry/trim/raw fault/version");$finish;
end
endmodule
