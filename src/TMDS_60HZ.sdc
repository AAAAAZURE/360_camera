
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}] -add

// PLL1 - 优化频率以减少功耗和噪声
create_generated_clock -name caux_clk -source [get_ports {clk}] -master_clock clk -divide_by 127 -multiply_by 24 [get_pins {Gowin_PLL_m0/PLL_inst/CLKOUT1}]
create_generated_clock -name mem_clk  -source [get_ports {clk}] -master_clock clk -divide_by 5   -multiply_by 24 [get_pins {Gowin_PLL_m0/PLL_inst/CLKOUT2}]  // 降低到240MHz

//PLL2 - 标准720p@60Hz：pixel=74.25MHz，serial=371.25MHz
create_generated_clock -name clk_tmds_5x -source [get_ports {clk}] -master_clock clk -divide_by 40  -multiply_by 297 [get_pins {u_tmds_pll/PLL_inst/CLKOUT0}]  // 371.25MHz
create_generated_clock -name clk_74_25   -source [get_ports {clk}] -master_clock clk -divide_by 200 -multiply_by 297 [get_pins {u_tmds_pll/PLL_inst/CLKOUT1}]  // 74.25MHz

//ddr pll
//create_generated_clock -name clk_x1 -source [get_nets {memory_clk}] -master_clock mem_clk -divide_by 4 -multiply_by 1 [get_pins {u_ddr3/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
create_generated_clock -name clk_x1 -source [get_pins {Gowin_PLL_m0/PLL_inst/CLKOUT2}] -master_clock mem_clk -divide_by 4 -multiply_by 1 [get_pins {u_ddr3/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]


//camera pclk - 降低摄像头时钟频率以减少功耗
create_clock -name cmos_pclk -period 13.468 -waveform {0 6.734} [get_ports {cmos_pclk}]  // 74.25MHz匹配显示时钟
create_clock -name cmos_vsync -period 16667 -waveform {0 8333} [get_ports {cmos_vsync}]  // 60Hz帧率
// cam1/cam2 pixel clocks (同等频率约束，避免未约束导致的时序抖动)
create_clock -name c1_pclk -period 13.468 -waveform {0 6.734} [get_ports {c1_pclk}]
create_clock -name c2_pclk -period 13.468 -waveform {0 6.734} [get_ports {c2_pclk}]

set_clock_groups -asynchronous -group [get_clocks {clk_tmds_5x}] 
                               -group [get_clocks {clk_74_25}] 
                               -group [get_clocks {clk_x1}] 
                               -group [get_clocks {caux_clk}] 
                               -group [get_clocks {mem_clk}] 
                               -group [get_clocks {cmos_pclk}] 
                               -group [get_clocks {c1_pclk}] 
                               -group [get_clocks {c2_pclk}] 
                               -group [get_clocks {cmos_vsync}] 
                               -group [get_clocks {clk}] 


//report_timing -hold -from_clock [get_clocks {clk*}] -to_clock [get_clocks {clk*}] -max_paths 25 -max_common_paths 1
//report_timing -setup -from_clock [get_clocks {clk*}] -to_clock [get_clocks {clk*}] -max_paths 25 -max_common_paths 1

// 屏蔽低速异步蓝牙/舵机端口的时序检查，避免影响主视频域
set_false_path -from [get_ports {bt_rxd bt_state}]
set_false_path -to   [get_ports {servo_pwm bt_txd bt_en}]
