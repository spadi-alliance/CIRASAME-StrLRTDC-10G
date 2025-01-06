# Clock definition
#create_clock -name clk_in -period 8 -waveform {0 4} [get_ports PHY_RX_CLK]
create_clock -period 8.000 -name clk_gmod -waveform {0.000 4.000} [get_pins u_BUFG/O]

#create_clock -period 7.692 -name clk_b2tt -waveform {0.000 3.846} [get_ports RJ45_TRG_P]
#set_input_jitter clk_b2tt 0.100

set_case_analysis 0 [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKINSEL]

create_generated_clock -name clk_sys [get_pins u_ClkMan_Inst/inst/mmcm_adv_inst/CLKOUT0]
create_generated_clock -name clk_indep [get_pins u_ClkMan_Inst/inst/mmcm_adv_inst/CLKOUT1]
create_generated_clock -name clk_spi [get_pins u_ClkMan_Inst/inst/mmcm_adv_inst/CLKOUT2]
create_generated_clock -name clk_machine [get_pins u_ClkMan_Inst/inst/mmcm_adv_inst/CLKOUT3]

create_generated_clock -name clk_slow [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT0]
create_generated_clock -name clk_fast [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT1]
create_generated_clock -name clk_tdc0 [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT2]
create_generated_clock -name clk_tdc1 [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT3]
create_generated_clock -name clk_tdc2 [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT4]
create_generated_clock -name clk_tdc3 [get_pins u_CdcmMan_Inst/inst/mmcm_adv_inst/CLKOUT5]

set_multicycle_path -setup -from [get_clocks clk_tdc3] -to [get_clocks clk_tdc0] 2

#create_generated_clock -name clk_mig_ui [get_pins u_MIG/u_mig_7series_0/u_mig_7series_0_mig/u_ddr3_infrastructure/gen_mmcm.mmcm_i/CLKFBOUT]
#create_generated_clock -name clk_mig_ui_out0 [get_pins u_MIG/u_mig_7series_0/u_mig_7series_0_mig/u_ddr3_infrastructure/gen_mmcm.mmcm_i/CLKOUT0]

#create_generated_clock -name clk_gmii1   [get_pins gen_pcspma[0].u_pcspma_Inst/core_support_i/core_clocking_i/mmcm_adv_inst/CLKOUT0]
#create_generated_clock -name clk_gmii2   [get_pins gen_pcspma[1].u_pcspma_Inst/core_support_i/core_clocking_i/mmcm_adv_inst/CLKOUT0]

#create_generated_clock -name clk_base     [get_pins u_ClkTdc_Inst/inst/plle2_adv_inst/CLKOUT0]
#create_generated_clock -name clk_tdc0     [get_pins u_ClkTdc_Inst/inst/plle2_adv_inst/CLKOUT1]
#create_generated_clock -name clk_tdc90    [get_pins u_ClkTdc_Inst/inst/plle2_adv_inst/CLKOUT2]
#create_generated_clock -name clk_tdc180   [get_pins u_ClkTdc_Inst/inst/plle2_adv_inst/CLKOUT3]
#create_generated_clock -name clk_tdc270   [get_pins u_ClkTdc_Inst/inst/plle2_adv_inst/CLKOUT4]

#set_multicycle_path -setup -from [get_clocks clk_tdc270] -to [get_clocks clk_tdc0] 2

set_clock_groups -name async_sys -asynchronous -group clk_sys -group {clk_fast clk_slow clk_tdc0 clk_tdc1 clk_tdc2 clk_tdc3} -group clk_indep -group clk_spi -group clk_machine -group GTX_REFCLK_P












