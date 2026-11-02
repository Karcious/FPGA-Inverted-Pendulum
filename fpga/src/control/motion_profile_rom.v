`timescale 1ns/1ps

module motion_profile_rom (
    input wire profile_id,
    input wire [2:0] segment,
    output reg signed [31:0] target_q16,
    output reg signed [31:0] slew_q16,
    output reg [15:0] dwell_ms,
    output reg last,
    output reg valid
);
    function signed [31:0] position;
        input integer degrees;
        reg signed [63:0] numerator;
        begin
            numerator = degrees;
            numerator = numerator * 64'sd1040 * 64'sd65536;
            position = numerator >= 0 ? (numerator + 180) / 360 :
                                        (numerator - 180) / 360;
        end
    endfunction
    function signed [31:0] speed;
        input integer degrees_per_second;
        reg signed [63:0] numerator;
        begin
            numerator = degrees_per_second;
            numerator = numerator * 64'sd1040 * 64'sd65536;
            speed = (numerator + 180000) / 360000;
        end
    endfunction

    always @* begin
        target_q16=0; slew_q16=0; dwell_ms=0; last=0; valid=1;
        case ({profile_id,segment})

            4'b0_000: begin target_q16=position(-45); slew_q16=speed(6);  dwell_ms=500; end
            4'b0_001: begin target_q16=position(45);  slew_q16=speed(10); dwell_ms=500; end
            4'b0_010: begin target_q16=position(0);   slew_q16=speed(6);  last=1; end

            4'b1_000: begin target_q16=position(90);  slew_q16=speed(8);  dwell_ms=800; end
            4'b1_001: begin target_q16=position(30);  slew_q16=speed(4);  dwell_ms=500; end
            4'b1_010: begin target_q16=position(-30); slew_q16=speed(8);  dwell_ms=500; end
            4'b1_011: begin target_q16=position(0);   slew_q16=speed(4);  last=1; end
            default: valid=0;
        endcase
    end
endmodule
