set_property IOSTANDARD LVCMOS33 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports {in_err[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {in_err[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {in_err[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {in_err[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {in_err[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {in_err[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {in_err[0]}]
set_property PACKAGE_PIN E3 [get_ports clk]
set_property PACKAGE_PIN U9 [get_ports {in_err[0]}]
set_property PACKAGE_PIN U8 [get_ports {in_err[1]}]
set_property PACKAGE_PIN R7 [get_ports {in_err[2]}]
set_property PACKAGE_PIN R6 [get_ports {in_err[3]}]
set_property PACKAGE_PIN R5 [get_ports {in_err[4]}]
set_property PACKAGE_PIN V7 [get_ports {in_err[5]}]
set_property PACKAGE_PIN V6 [get_ports {in_err[6]}]
set_property PACKAGE_PIN T8 [get_ports err]
set_property PACKAGE_PIN V9 [get_ports mem_busy]

set_property PACKAGE_PIN R8 [get_ports resetn]
set_property PACKAGE_PIN T6 [get_ports trap]
set_property PACKAGE_PIN T5 [get_ports mem_ready]
set_property PACKAGE_PIN T4 [get_ports mem_valid]

set_property PACKAGE_PIN U6 [get_ports led4]
set_property PACKAGE_PIN V4 [get_ports led5]
set_property PACKAGE_PIN U3 [get_ports mem_valid1]
set_property IOSTANDARD LVCMOS33 [get_ports err]

set_property IOSTANDARD LVCMOS33 [get_ports mem_valid]
set_property IOSTANDARD LVCMOS33 [get_ports mem_valid1]

set_property IOSTANDARD LVCMOS33 [get_ports led4]
set_property IOSTANDARD LVCMOS33 [get_ports led5]
set_property IOSTANDARD LVCMOS33 [get_ports mem_busy]
set_property IOSTANDARD LVCMOS33 [get_ports mem_ready]
set_property IOSTANDARD LVCMOS33 [get_ports resetn]
set_property IOSTANDARD LVCMOS33 [get_ports trap]

set_property IOSTANDARD LVCMOS33 [get_ports {anodes[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {anodes[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {anodes[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {anodes[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {anodes[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {anodes[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {anodes[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {anodes[0]}]



set_property IOSTANDARD LVCMOS33 [get_ports {seg_display[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_display[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_display[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_display[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_display[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_display[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_display[0]}]

set_property PACKAGE_PIN N6 [get_ports {anodes[0]}]
set_property PACKAGE_PIN M6 [get_ports {anodes[1]}]
set_property PACKAGE_PIN M3 [get_ports {anodes[2]}]
set_property PACKAGE_PIN N5 [get_ports {anodes[3]}]
set_property PACKAGE_PIN N2 [get_ports {anodes[4]}]
set_property PACKAGE_PIN N4 [get_ports {anodes[5]}]
set_property PACKAGE_PIN L1 [get_ports {anodes[6]}]
set_property PACKAGE_PIN M1 [get_ports {anodes[7]}]


set_property PACKAGE_PIN L3 [get_ports {seg_display[6]}]
set_property PACKAGE_PIN N1 [get_ports {seg_display[5]}]
set_property PACKAGE_PIN L5 [get_ports {seg_display[4]}]
set_property PACKAGE_PIN L4 [get_ports {seg_display[3]}]
set_property PACKAGE_PIN K3 [get_ports {seg_display[2]}]
set_property PACKAGE_PIN M2 [get_ports {seg_display[1]}]
set_property PACKAGE_PIN L6 [get_ports {seg_display[0]}]

create_clock -period 10.0 [get_ports clk]
#create_clock -period 10.0 [get_ports clka]

#set_input_delay -clock [get_clocks clk] -max 0.5 [get_ports in_err]

#set_output_delay -clock [get_clocks clk] -max 0.2 [get_ports {err trap mem_busy}]

#set_clock_uncertainty 0.1 [get_clocks clk]


#set_false_path -from [get_ports in_err] -to [get_ports {err trap mem_busy}]

#set_multicycle_path 2 -from [get_ports in_err] -to [get_ports {err trap mem_busy}]

set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets clk]
