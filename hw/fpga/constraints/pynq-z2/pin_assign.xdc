# Copyright 2022 EPFL
# Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
# SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1

# For pinout of the PYNQ-Z2 please refer to 
# https://us1.discourse-cdn.com/flex019/uploads/pynq1/original/2X/5/5b969c46185b0799d848915df3762fce368bf55d.png

# Clock signal
set_property -dict {PACKAGE_PIN H16 IOSTANDARD LVCMOS33} [get_ports clk_i]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets jtag_tck_i_IBUF]

set_property -dict {PACKAGE_PIN L19 IOSTANDARD LVCMOS33} [get_ports rst_i]

# LEDs
set_property -dict {PACKAGE_PIN M14 IOSTANDARD LVCMOS33} [get_ports rst_led_o]
set_property -dict {PACKAGE_PIN N16 IOSTANDARD LVCMOS33} [get_ports clk_led_o]
set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS33} [get_ports exit_valid_o]
set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS33} [get_ports exit_value_o]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets rst_led_OBUF]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_out_OBUF]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_led_OBUF]

# Switches
set_property -dict {PACKAGE_PIN M19 IOSTANDARD LVCMOS33} [get_ports execute_from_flash_i] ; # IO_L7P_T1_AD2P_35 Sch=sw[1]
set_property -dict {PACKAGE_PIN M20 IOSTANDARD LVCMOS33} [get_ports boot_select_i] ; # IO_L7N_T1_AD2N_35 Sch=sw[0]

# FLASH
# QSPI
# Q0 / MOSI
# Q1 / MISO
# Q2 / nWP
# Q3 / nHLD
set_property -dict {PACKAGE_PIN U18 IOSTANDARD LVCMOS33} [get_ports spi_flash_csb_o] ; # Pmoda[4]
set_property -dict {PACKAGE_PIN Y18 IOSTANDARD LVCMOS33} [get_ports spi_flash_sck_o] ; # Pmoda[0]
set_property -dict {PACKAGE_PIN U19 IOSTANDARD LVCMOS33} [get_ports {spi_flash_sd_io[0]}] ; # Pmoda[5]
set_property -dict {PACKAGE_PIN Y19 IOSTANDARD LVCMOS33} [get_ports {spi_flash_sd_io[1]}] ; # Pmoda[1]
set_property -dict {PACKAGE_PIN W18 IOSTANDARD LVCMOS33} [get_ports {spi_flash_sd_io[2]}] ; # Pmoda[6]
set_property -dict {PACKAGE_PIN Y16 IOSTANDARD LVCMOS33} [get_ports {spi_flash_sd_io[3]}] ; # Pmoda[2]

# UART
set_property -dict {PACKAGE_PIN W14 IOSTANDARD LVCMOS33} [get_ports uart_tx_o] ; # Pmodb[0]
set_property -dict {PACKAGE_PIN V16 IOSTANDARD LVCMOS33} [get_ports uart_rx_i] ; # Pmodb[4]

# JTAG
set_property -dict {PACKAGE_PIN Y14 IOSTANDARD LVCMOS33} [get_ports jtag_tdi_i] ; # Pmob[1]
set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports jtag_tdo_o] ; # Pmodb[6]
set_property -dict {PACKAGE_PIN T11 IOSTANDARD LVCMOS33} [get_ports jtag_tms_i] ; # Pmodb[2]
set_property -dict {PACKAGE_PIN W16 IOSTANDARD LVCMOS33} [get_ports jtag_tck_i] ; # Pmodb[5]
set_property -dict {PACKAGE_PIN W19 IOSTANDARD LVCMOS33} [get_ports jtag_trst_ni] ; # Pmoda[7]

# SPI
set_property -dict {PACKAGE_PIN T15 IOSTANDARD LVCMOS33} [get_ports {spi_csb_0_o}] ;
set_property -dict {PACKAGE_PIN T16 IOSTANDARD LVCMOS33} [get_ports {spi_csb_1_o}] ;
set_property -dict {PACKAGE_PIN N17 IOSTANDARD LVCMOS33} [get_ports spi_sck_o] ; # arduino_direct_spi_sck_io -- CSK
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33} [get_ports {spi_sd_io[0]}] ; # arduino_direct_spi_io0_io -- MOSI
set_property -dict {PACKAGE_PIN P18 IOSTANDARD LVCMOS33} [get_ports {spi_sd_io[1]}] ; # arduino_direct_spi_io1_io -- MISO
set_property -dict {PACKAGE_PIN V13 IOSTANDARD LVCMOS33} [get_ports {spi_sd_io[2]}] ; # arduino_gpio_tri_io[12]
set_property -dict {PACKAGE_PIN V15 IOSTANDARD LVCMOS33} [get_ports {spi_sd_io[3]}] ; # arduino_gpio_tri_io[13]

# GPIOs
set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS33} [get_ports {gpio_io[0]}] ; # arduino_gpio_tri_io[0]
set_property -dict {PACKAGE_PIN U10 IOSTANDARD LVCMOS33} [get_ports {gpio_io[1]}] ; # rpi_gpio_tri_io[11]
set_property -dict {PACKAGE_PIN T5 IOSTANDARD LVCMOS33 PULLUP true} [get_ports {gpio_io[2]}] ; # rpi_gpio_tri_io[5]
set_property -dict {PACKAGE_PIN V18 IOSTANDARD LVCMOS33} [get_ports {gpio_io[3]}] ; # rpi_gpio_tri_io[16]
set_property -dict {PACKAGE_PIN W8 IOSTANDARD LVCMOS33} [get_ports {gpio_io[4]}] ; # rpi_gpio_tri_io[7]
set_property -dict {PACKAGE_PIN W9 IOSTANDARD LVCMOS33} [get_ports {gpio_io[5]}] ; # arduino_gpio_tri_io[1]
set_property -dict {PACKAGE_PIN U13 IOSTANDARD LVCMOS33} [get_ports {gpio_io[6]}] ; # rpi_gpio_tri_io[3]
set_property -dict {PACKAGE_PIN Y8 IOSTANDARD LVCMOS33} [get_ports {gpio_io[7]}] ; # rpi_gpio_tri_io[1]
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS33 PULLUP true} [get_ports {gpio_io[8]}] ; # rpi_gpio_tri_io[2]
set_property -dict {PACKAGE_PIN U12 IOSTANDARD LVCMOS33 PULLUP true} [get_ports {gpio_io[9]}] ; # rpi_gpio_tri_io[7]
set_property -dict {PACKAGE_PIN U17 IOSTANDARD LVCMOS33} [get_ports {gpio_io[10]}] ; 

#UART2
set_property -dict {PACKAGE_PIN U8 IOSTANDARD LVCMOS33} [get_ports {uart2_tx_o}] ; # rpi_gpio_tri_io[14]
set_property -dict {PACKAGE_PIN V7 IOSTANDARD LVCMOS33} [get_ports {uart2_rx_i}] ; # rpi_gpio_tri_io[19]


# SPI SLAVE -- SE PODRÍA ELIMINAR
set_property -dict {PACKAGE_PIN Y9 IOSTANDARD LVCMOS33} [get_ports {spi_slave_sck_io}] ; # rpi_gpio_tri_io[9]
set_property -dict {PACKAGE_PIN A20 IOSTANDARD LVCMOS33} [get_ports {spi_slave_cs_io}] ; # rpi_gpio_tri_io[6]
set_property -dict {PACKAGE_PIN B19 IOSTANDARD LVCMOS33} [get_ports {spi_slave_miso_io}] ; # arduino_gpio_tri_io[2]
set_property -dict {PACKAGE_PIN B20 IOSTANDARD LVCMOS33} [get_ports {spi_slave_mosi_io}] ; # arduino_gpio_tri_io[3]