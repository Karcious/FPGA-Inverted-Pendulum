cd [file normalize [file join [file dirname [info script]] ..]]
set_device GW2A-LV55PG484C8/I7 -device_version C
set_option -use_sspi_as_gpio 1
set_option -top_module pend_top
add_file src/common/crc8.v
add_file src/common/uart_rx.v
add_file src/common/uart_tx.v
add_file src/debug/param_regs.v
add_file src/debug/capture.v
add_file src/debug/dbg_proto.v
add_file src/debug/dbg_top.v
add_file src/control/pend_top.v
add_file src/control/adc_capture.v
add_file src/control/encoder_decode.v
add_file src/control/pwm_gen.v
add_file src/control/pid_q16.v
add_file src/control/fuzzy_pid_q16.v
add_file src/control/safety_guard.v
add_file src/control/position_feedback.v
add_file src/control/friction_state.v
add_file src/control/swing_once.v
add_file src/control/start_button.v
add_file src/control/key_motion_target.v
add_file src/control/motion_profile_rom.v
add_file src/control/motion_profile_sequencer.v
add_file src/control/spin_trajectory.v
add_file src/control/pend_top.cst
add_file src/fpga_project.sdc
set_option -place_option 1
set_option -route_option 1
set_option -max_fanout 40
add_file -type verilog src/control/quiet_guard.v
add_file -type verilog src/control/tof_receiver.v
run all
