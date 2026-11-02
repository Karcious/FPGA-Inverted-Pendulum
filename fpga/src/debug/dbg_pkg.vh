`define DBG_MAGIC0      8'hAA
`define DBG_MAGIC1      8'h55
`define DBG_RESP_FLAG   8'h80

`define CMD_PING        8'h01
`define CMD_SET_PARAM   8'h10
`define CMD_GET_PARAM   8'h11
`define CMD_DUMP_PARAMS 8'h12
`define CMD_ARM_CAPTURE 8'h20
`define CMD_READ_CAPTURE 8'h21
`define CMD_CAP_STATUS  8'h22
`define CMD_DISARM      8'h23
`define CMD_READ_LIVE   8'h24
`define CMD_READ_FUZZY  8'h0A

`define CAPST_ARMED     3'd0
`define CAPST_DONE      3'd1

`define DBG_MAX_PAYLOAD 16

`define P_MAGIC         6'd0
`define P_CTRL_HZ       6'd1
`define P_PWM_HZ        6'd2
`define P_KP_BAL        6'd3
`define P_KD_BAL        6'd4
`define P_KP_ARM        6'd5
`define P_KI_ARM        6'd6
`define P_THETA_SWITCH  6'd7
`define P_OMEGA_SWITCH  6'd8

`define P_FILT_SEL      6'd9
`define P_DUTY_MIN      6'd10
`define P_DUTY_MAX      6'd11
`define P_SWING_GAIN    6'd12
`define P_TARGET_POS    6'd13

`define P_POS_SLEW      6'd14

`define P_TRIM_THETA    6'd15

`define P_OPENLOOP      6'd16

`define P_DUTY_TEST     6'd17

`define P_CENTER_ANGLE  6'd18

`define P_MOTOR_EN      6'd19

`define P_KD_ARM        6'd20
`define P_CAPTURE_LOG2  6'd21
`define P_FRICTION_MODE 6'd22
`define P_MOVING_COMP   6'd23
`define P_SWING_DUTY    6'd24
`define P_SWING_MS      6'd25
`define P_SWING_TIMEOUT 6'd26
`define P_SWING_TRAVEL  6'd27
`define P_HANG_ANGLE    6'd28
`define P_PUMP_START    6'd29
`define P_PUMP_MAX      6'd30
`define P_STATUS        6'd31

`define P_SWING_KP 6'd12
`define P_SWING_KD 6'd14
`define P_SWING_TARGET 6'd29
`define P_CAPTURE_MS 6'd30

`define P_WAITUP_BRAKE 6'd32

`define P_GAIN_A_KP    6'd33
`define P_GAIN_A_KD    6'd34
`define P_GAIN_B_KP    6'd35
`define P_GAIN_B_KD    6'd36
`define P_PHASE_A_MS   6'd37
`define P_PHASE_B_MS   6'd38

`define P_SETTLE_DAMP  6'd39

`define FILT_FUZZY_BIT 8

`define P_MOVE_SLEW     6'd40

`define P_PROF_SLEW     6'd41

`define P_MOVE_ACC      6'd42

`define P_SPIN_W        6'd43

`define P_SPIN_KP       6'd44
`define P_SPIN_KI       6'd45

`define P_SPIN_ACC      6'd46

`define P_HOME_RAMP     6'd47

`define P_ARM_MAX       6'd48

`define P_MOVE_DEC      6'd49

`define P_COMP_SHIFT  6'd50

`define P_TRAVEL_FOLLOW 6'd51
