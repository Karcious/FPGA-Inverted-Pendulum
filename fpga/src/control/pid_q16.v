module pid_q16 #(

    parameter integer SEPARATE_D = 0
)(

    input  wire               clk,
    input  wire               rst_n,

    input  wire               en,
    input  wire               clear,

    input  wire signed [31:0] target,
    input  wire signed [31:0] actual,
    input  wire signed [31:0] d_input,
    input  wire               d_clear,

    input  wire signed [31:0] kp,
    input  wire signed [31:0] ki,
    input  wire signed [31:0] kd,

    input  wire signed [31:0] out_max,
    input  wire signed [31:0] out_min,

    input  wire               en_dlpf,
    input  wire               en_dspan2,

    input  wire               en_dspan4,
    input  wire        [1:0]  lpf_shift,

    output reg  signed [31:0] out
);

    function signed [31:0] sat32;
        input signed [63:0] v;
        begin
            if      (v >  64'sd2147483647) sat32 =  32'sd2147483647;
            else if (v < -64'sd2147483648) sat32 = -32'sd2147483648;
            else                           sat32 = v[31:0];
        end
    endfunction

    wire signed [31:0] err_now = target - actual;
    wire signed [31:0] d_now = SEPARATE_D ? d_input : err_now;
    wire derivative_clear = clear || ((SEPARATE_D!=0) && d_clear);
    reg signed [31:0] deriv_d;

    reg signed [31:0] err_d;
    reg signed [31:0] err_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            err_d  <= 32'sd0;
            err_d1 <= 32'sd0;
            deriv_d <= 32'sd0;
        end else if (en) begin
            err_d  <= err_now;
            deriv_d <= d_now;

            err_d1 <= derivative_clear ? d_now : deriv_d;
        end
    end

    reg signed [31:0] err_d2;

    reg signed [31:0] err_d3;
    reg signed [31:0] err_d4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)  err_d2 <= 32'sd0;
        else if (en) err_d2 <= derivative_clear ? d_now : err_d1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)  err_d3 <= 32'sd0;
        else if (en) err_d3 <= derivative_clear ? d_now : err_d2;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)  err_d4 <= 32'sd0;
        else if (en) err_d4 <= derivative_clear ? d_now : err_d3;
    end

    wire signed [31:0] d_err1 = deriv_d - err_d1;
    wire signed [31:0] d_err2 = (deriv_d - err_d2) >>> 1;

    wire signed [31:0] d_err4 = (deriv_d - err_d4) >>> 2;

    wire signed [31:0] d_err  = en_dspan4 ? d_err4 :
                                (en_dspan2 ? d_err2 : d_err1);

    reg signed [63:0] pr_p;
    reg signed [63:0] pr_i;
    reg signed [63:0] pr_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pr_p <= 64'sd0;
            pr_i <= 64'sd0;
            pr_d <= 64'sd0;
        end else begin
            pr_p <= kp * err_d;
            pr_i <= ki * err_d;
            pr_d <= kd * d_err;
        end
    end

    reg signed [31:0] pp_s;
    reg signed [31:0] dd_s;
    reg signed [31:0] ii_s;

    reg signed [31:0] integ;

    wire sat_hi = (out >= out_max) && (ii_s > 32'sd0);
    wire sat_lo = (out <= out_min) && (ii_s < 32'sd0);

    wire signed [39:0] integ_raw = {{8{integ[31]}}, integ} + {{8{ii_s[31]}}, ii_s};
    wire signed [39:0] range_hi  = {{8{out_max[31]}}, out_max};
    wire signed [39:0] range_lo  = {{8{out_min[31]}}, out_min};

    wire signed [39:0] integ_lim =
        (integ_raw > range_hi) ? range_hi :
        (integ_raw < range_lo) ? range_lo : integ_raw;

    wire signed [31:0] integ_next = (sat_hi || sat_lo) ? integ : integ_lim[31:0];

    wire signed [39:0] integ_sum = clear ? 40'sd0 : {{8{integ[31]}}, integ};
    reg  signed [39:0] sum_r;
    reg                v1, v2, v3, v4, v5;

    reg  signed [31:0] d_lpf;

    wire [4:0] lpf_k = 5'd3 - {3'd0, lpf_shift};

    wire signed [63:0] d_lpf_e = $signed({{32{d_lpf[31]}}, d_lpf});
    wire signed [63:0] dd_s_e  = $signed({{32{dd_s[31]}},  dd_s});
    wire signed [63:0] lpf_old = (d_lpf_e <<< 4) - (d_lpf_e <<< lpf_k);
    wire signed [63:0] lpf_new = (dd_s_e  <<< lpf_k);
    wire signed [31:0] d_next  = sat32((lpf_old + lpf_new) >>> 4);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)     d_lpf <= 32'sd0;
        else if (derivative_clear) d_lpf <= 32'sd0;
        else if (v3)    d_lpf <= d_next;
    end

    wire signed [31:0] dd_used = en_dlpf ? d_lpf : dd_s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pp_s  <= 32'sd0;
            dd_s  <= 32'sd0;
            ii_s  <= 32'sd0;
            integ <= 32'sd0;
            sum_r <= 40'sd0;
            v1    <= 1'b0;
            v2    <= 1'b0;
            v3    <= 1'b0;
            v4    <= 1'b0;
            v5    <= 1'b0;
        end else begin
            v1 <= en;
            v2 <= v1;
            v3 <= v2;
            v4 <= v3;
            v5 <= v4;

            if (v2) begin
                pp_s <= sat32(pr_p >>> 16);
                dd_s <= sat32(pr_d >>> 16);
                ii_s <= sat32(pr_i >>> 16);
            end

            if (clear) begin
                integ <= 32'sd0;
            end else if (v3) begin
                integ <= integ_next;
            end

            if (v4) begin
                sum_r <= {{8{pp_s[31]}}, pp_s}
                       + {{8{dd_used[31]}}, dd_used}
                       + integ_sum;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out <= 32'sd0;
        end else if (v5) begin
            if (sum_r > range_hi)      out <= out_max;
            else if (sum_r < range_lo) out <= out_min;
            else                       out <= sum_r[31:0];
        end
    end

endmodule
