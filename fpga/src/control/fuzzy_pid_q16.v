`timescale 1ns / 1ps
module fuzzy_pid_q16 #(
    parameter integer E_STEP_SHIFT=2,
    parameter integer EC_STEP_SHIFT=0,
    parameter integer TUNE_ENABLE=1,
    parameter integer SEPARATE_D=0,
    parameter [32:0] E_DEADBAND_Q16=33'd131072,
    parameter [32:0] EC_DEADBAND_Q16=33'd32768,
    parameter integer EC_FILTER_SHIFT=2,

    parameter [9:0] KP_SLEW_Q8=10'd1,
    parameter [9:0] KD_SLEW_Q8=10'd2
)(
    input wire clk,rst_n,en,clear,fuzzy_enable,
    input wire signed [31:0] target,actual,kp,ki,kd,out_max,out_min,
    input wire signed [31:0] d_input,
    input wire d_clear,
    input wire en_dlpf,en_dspan2,en_dspan4,
    input wire [1:0] lpf_shift,
    output wire signed [31:0] out,
    output reg signed [31:0] kp_eff,ki_eff,kd_eff,
    output wire [9:0] kp_scale_dbg,kd_scale_dbg,
    output wire signed [32:0] ec_filtered_dbg,
    output wire sample_done
);
    localparam [2:0] NB=0,NS=1,ZE=2,PS=3,PB=4;
    localparam [32:0] E_STEP_Q16=33'd65536 << E_STEP_SHIFT;
    localparam [32:0] EC_STEP_Q16=33'd65536 << EC_STEP_SHIFT;
    wire fuzzy_active=fuzzy_enable && (TUNE_ENABLE!=0);
    wire signed [31:0] err_input=target-actual;
    reg signed [31:0] err_prev;
    reg signed [32:0] err_now,ec_ext,ec_filter_q;

    reg signed [32:0] ec_pre_q;
    reg history_ready,clear_pending,clear_q;
    reg [4:0] stage;
    reg signed [31:0] target_q,actual_q,kp_q,ki_q,kd_q,max_q,min_q;
    reg signed [31:0] d_input_q;
    reg d_clear_q;
    reg dlpf_q,dspan2_q,dspan4_q;
    reg [1:0] lpf_q;
    wire signed [32:0] ec_raw={err_input[31],err_input}-{err_prev[31],err_prev};
    wire signed [33:0] ec_difference={ec_raw[32],ec_raw}-{ec_filter_q[32],ec_filter_q};

    wire signed [33:0] ec_adjustment=ec_difference >>> EC_FILTER_SHIFT;
    wire signed [33:0] ec_filter_next=$signed({ec_filter_q[32],ec_filter_q})+ec_adjustment;
    function signed [32:0] soft_deadband;
        input signed [32:0] value;
        input [32:0] width;
        reg [32:0] magnitude,remainder;
        begin
            magnitude=value[32] ? -value : value;
            remainder=(magnitude>width) ? magnitude-width : 33'd0;
            soft_deadband=value[32] ? -$signed(remainder) : $signed(remainder);
        end
    endfunction
    wire signed [32:0] ec_band=soft_deadband(ec_pre_q,EC_DEADBAND_Q16);
    wire [32:0] e_abs=err_now[32] ? -err_now : err_now;
    wire [32:0] ec_abs=ec_band[32] ? -ec_band : ec_band;
    wire [32:0] e_over_step=e_abs-E_STEP_Q16;
    wire [32:0] ec_over_step=ec_abs-EC_STEP_Q16;
    wire [40:0] e_ratio0={e_abs,8'd0} >> (16+E_STEP_SHIFT);
    wire [40:0] e_ratio1={e_over_step,8'd0} >> (16+E_STEP_SHIFT);
    wire [40:0] ec_ratio0={ec_abs,8'd0} >> (16+EC_STEP_SHIFT);
    wire [40:0] ec_ratio1={ec_over_step,8'd0} >> (16+EC_STEP_SHIFT);
    reg [2:0] e_lo_next,e_hi_next,ec_lo_next,ec_hi_next;
    reg [8:0] e_mu_hi_next,ec_mu_hi_next;
    always @* begin
        e_lo_next=ZE; e_hi_next=ZE; e_mu_hi_next=0;
        if(e_abs >= (E_STEP_Q16<<1)) begin
            e_lo_next=err_now[32] ? NB : PB; e_hi_next=e_lo_next;
        end else if(e_abs<E_STEP_Q16) begin
            e_hi_next=err_now[32] ? NS : PS; e_mu_hi_next=e_ratio0[8:0];
        end else begin
            e_lo_next=err_now[32] ? NS : PS; e_hi_next=err_now[32] ? NB : PB;
            e_mu_hi_next=e_ratio1[8:0];
        end
        ec_lo_next=ZE; ec_hi_next=ZE; ec_mu_hi_next=0;

        if(ec_abs >= (EC_STEP_Q16<<1)) begin
            ec_lo_next=ec_band[32] ? NB : PB; ec_hi_next=ec_lo_next;
        end else if(ec_abs<EC_STEP_Q16) begin
            ec_hi_next=ec_band[32] ? NS : PS; ec_mu_hi_next=ec_ratio0[8:0];
        end else begin
            ec_lo_next=ec_band[32] ? NS : PS; ec_hi_next=ec_band[32] ? NB : PB;
            ec_mu_hi_next=ec_ratio1[8:0];
        end
    end
    reg [2:0] e_lo,e_hi,ec_lo,ec_hi;
    reg [8:0] e_mu_hi,ec_mu_hi;
    wire [8:0] e_mu_lo=9'd256-e_mu_hi,ec_mu_lo=9'd256-ec_mu_hi;

    function [1:0] kp_rule;
        input [2:0] e_set,ec_set;
        begin
            kp_rule=0;
            case(e_set)
                NB: case(ec_set) NB:kp_rule=3; NS:kp_rule=2; default:kp_rule=0; endcase
                NS: case(ec_set) NB:kp_rule=2; NS:kp_rule=1; default:kp_rule=0; endcase
                PS: case(ec_set) PS:kp_rule=1; PB:kp_rule=2; default:kp_rule=0; endcase
                PB: case(ec_set) PS:kp_rule=2; PB:kp_rule=3; default:kp_rule=0; endcase
                default:kp_rule=0;
            endcase
        end
    endfunction

    function [1:0] kd_rule;
        input [2:0] e_set,ec_set;
        begin
            kd_rule=0;
            case(ec_set)
                NB,PB: kd_rule=(e_set==NB || e_set==PB) ? 2 : 3;
                NS,PS: kd_rule=(e_set==NB || e_set==PB) ? 1 : 2;
                default:kd_rule=0;
            endcase
        end
    endfunction

    function [27:0] weighted_scale;
        input [17:0] weight;
        input [1:0] rule_value;
        input derivative;
        reg [27:0] w;
        begin
            w={10'd0,weight};
            if(derivative) case(rule_value)
                1:weighted_scale=(w<<8)+(w<<4)+(w<<2)+w;
                2:weighted_scale=(w<<8)+(w<<5)+(w<<3)+(w<<1)+w;
                3:weighted_scale=(w<<8)+(w<<6);
                default:weighted_scale=w<<8;
            endcase
            else case(rule_value)
                1:weighted_scale=(w<<8)+(w<<3)+(w<<1)+w;
                2:weighted_scale=(w<<8)+(w<<4)+(w<<2)+w;
                3:weighted_scale=(w<<8)+(w<<5);
                default:weighted_scale=w<<8;
            endcase
        end
    endfunction
    function [9:0] slew_scale;
        input [9:0] previous,requested,maximum,step_size;
        reg [9:0] bounded;
        begin
            bounded=(requested<256) ? 10'd256 : (requested>maximum ? maximum : requested);
            if(bounded>previous && bounded-previous>step_size) slew_scale=previous+step_size;
            else if(previous>bounded && previous-bounded>step_size) slew_scale=previous-step_size;
            else slew_scale=bounded;
        end
    endfunction
    function signed [31:0] saturate_gain;
        input signed [42:0] product;
        reg signed [42:0] shifted;
        begin
            shifted=product>>>8;
            if(shifted>43'sd2147483647) saturate_gain=32'sh7fffffff;
            else if(shifted < -43'sd2147483648) saturate_gain=32'sh80000000;
            else saturate_gain=shifted[31:0];
        end
    endfunction
    reg [17:0] w00,w01,w10,w11;
    reg [1:0] kp_r00,kp_r01,kp_r10,kp_r11,kd_r00,kd_r01,kd_r10,kd_r11;
    reg [27:0] kp_weighted00,kp_weighted01,kp_weighted10,kp_weighted11;
    reg [27:0] kd_weighted00,kd_weighted01,kd_weighted10,kd_weighted11;
    reg [27:0] kp_scale_sum,kd_scale_sum;
    reg [9:0] kp_scale_uq8,kd_scale_uq8,kp_scale_bits,kd_scale_bits;
    reg signed [42:0] kp_product,kp_shift,kd_product,kd_shift;
    assign kp_scale_dbg=fuzzy_active ? kp_scale_uq8 : 10'd256;
    assign kd_scale_dbg=fuzzy_active ? kd_scale_uq8 : 10'd256;
    assign ec_filtered_dbg=fuzzy_active ? ec_filter_q : 33'sd0;
    assign sample_done=fuzzy_active ? stage==18 : en;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            stage<=0; err_prev<=0; err_now<=0; ec_ext<=0; ec_filter_q<=0; ec_pre_q<=0;
            history_ready<=0; clear_pending<=1; clear_q<=1;
            kp_scale_uq8<=256; kd_scale_uq8<=256;
            kp_eff<=0; ki_eff<=0; kd_eff<=0;
        end else begin
            if(clear) begin
                clear_pending<=1; ec_filter_q<=0;
                kp_scale_uq8<=256; kd_scale_uq8<=256;
            end
            if(en) begin
                err_prev<=err_input;
                err_now<=soft_deadband({err_input[31],err_input},E_DEADBAND_Q16);
                target_q<=target; actual_q<=actual; kp_q<=kp; ki_q<=ki; kd_q<=kd;
                 d_input_q<=d_input;
                 d_clear_q<=d_clear;
                max_q<=out_max; min_q<=out_min;
                clear_q<=clear || clear_pending; clear_pending<=0;
                dlpf_q<=en_dlpf; dspan2_q<=en_dspan2; dspan4_q<=en_dspan4; lpf_q<=lpf_shift;
                history_ready<=fuzzy_active;
                if(clear || clear_pending || !history_ready || !fuzzy_active) begin
                    ec_ext<=0; ec_filter_q<=0; ec_pre_q<=0;
                    kp_scale_uq8<=256; kd_scale_uq8<=256;
                end else begin
                    ec_filter_q<=ec_filter_next[32:0];

                    ec_pre_q<=ec_filter_next[32:0];
                end
                stage<=fuzzy_active ? 1 : 0;
                if(!fuzzy_active) begin kp_eff<=kp; ki_eff<=ki; kd_eff<=kd; end
            end else begin
                if(stage!=0) stage <= (stage==18) ? 0 : stage+1'b1;
                case(stage)
                    1: begin

                        ec_ext<=ec_band;
                        e_lo<=e_lo_next; e_hi<=e_hi_next; ec_lo<=ec_lo_next; ec_hi<=ec_hi_next;
                        e_mu_hi<=e_mu_hi_next; ec_mu_hi<=ec_mu_hi_next;
                    end
                    2: begin
                        w00<=e_mu_lo*ec_mu_lo; w01<=e_mu_lo*ec_mu_hi;
                        w10<=e_mu_hi*ec_mu_lo; w11<=e_mu_hi*ec_mu_hi;
                        kp_r00<=kp_rule(e_lo,ec_lo); kp_r01<=kp_rule(e_lo,ec_hi);
                        kp_r10<=kp_rule(e_hi,ec_lo); kp_r11<=kp_rule(e_hi,ec_hi);
                        kd_r00<=kd_rule(e_lo,ec_lo); kd_r01<=kd_rule(e_lo,ec_hi);
                        kd_r10<=kd_rule(e_hi,ec_lo); kd_r11<=kd_rule(e_hi,ec_hi);
                    end
                    3: begin
                        kp_weighted00<=weighted_scale(w00,kp_r00,0); kp_weighted01<=weighted_scale(w01,kp_r01,0);
                        kp_weighted10<=weighted_scale(w10,kp_r10,0); kp_weighted11<=weighted_scale(w11,kp_r11,0);
                        kd_weighted00<=weighted_scale(w00,kd_r00,1); kd_weighted01<=weighted_scale(w01,kd_r01,1);
                        kd_weighted10<=weighted_scale(w10,kd_r10,1); kd_weighted11<=weighted_scale(w11,kd_r11,1);
                    end
                    4: begin
                        kp_scale_sum<=kp_weighted00+kp_weighted01+kp_weighted10+kp_weighted11;
                        kd_scale_sum<=kd_weighted00+kd_weighted01+kd_weighted10+kd_weighted11;
                    end
                    5: begin
                        if(clear || clear_pending || clear_q) begin kp_scale_uq8<=256; kd_scale_uq8<=256; end
                        else begin
                            kp_scale_uq8<=slew_scale(kp_scale_uq8,(kp_scale_sum+28'd32768)>>16,10'd288,KP_SLEW_Q8);
                            kd_scale_uq8<=slew_scale(kd_scale_uq8,(kd_scale_sum+28'd32768)>>16,10'd320,KD_SLEW_Q8);
                        end
                    end
                    6: begin
                        kp_product<=0; kp_shift<={{11{kp_q[31]}},kp_q}; kp_scale_bits<=kp_scale_uq8;
                        kd_product<=0; kd_shift<={{11{kd_q[31]}},kd_q}; kd_scale_bits<=kd_scale_uq8;
                    end
                    7,8,9,10,11,12,13,14,15,16: begin
                        kp_product<=kp_product+(kp_scale_bits[0] ? kp_shift : 43'sd0);
                        kp_shift<=kp_shift<<<1; kp_scale_bits<=kp_scale_bits>>1;
                        kd_product<=kd_product+(kd_scale_bits[0] ? kd_shift : 43'sd0);
                        kd_shift<=kd_shift<<<1; kd_scale_bits<=kd_scale_bits>>1;
                    end
                    17: begin
                        kp_eff <= (clear || clear_pending || clear_q) ? kp_q : saturate_gain(kp_product);
                        ki_eff <= ki_q;
                        kd_eff <= (clear || clear_pending || clear_q) ? kd_q : saturate_gain(kd_product);
                    end
                endcase
            end
        end
    end
    pid_q16 #(.SEPARATE_D(SEPARATE_D)) u_pid (
        .clk(clk),.rst_n(rst_n),.en(fuzzy_active ? stage==18 : en),
        .clear(fuzzy_active ? clear || clear_pending || clear_q : clear),
        .target(fuzzy_active ? target_q : target),.actual(fuzzy_active ? actual_q : actual),
        .d_input(fuzzy_active ? d_input_q : d_input),
        .d_clear(fuzzy_active ? d_clear_q : d_clear),
        .kp(fuzzy_active ? kp_eff : kp),.ki(fuzzy_active ? ki_eff : ki),.kd(fuzzy_active ? kd_eff : kd),
        .out_max(fuzzy_active ? max_q : out_max),.out_min(fuzzy_active ? min_q : out_min),
        .en_dlpf(fuzzy_active ? dlpf_q : en_dlpf),.en_dspan2(fuzzy_active ? dspan2_q : en_dspan2),
        .en_dspan4(fuzzy_active ? dspan4_q : en_dspan4),.lpf_shift(fuzzy_active ? lpf_q : lpf_shift),.out(out)
    );
endmodule
