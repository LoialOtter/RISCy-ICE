/*
 * This file is part of the RISCy-ICE project.
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Matthew Peters - matthew@ottersoft.ca
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

`include "globals.vh"

module base_system 
     #(
       parameter WB_ADDR_WIDTH  = 24,
       parameter CLK_FREQ       = 16000000,
       localparam WB_DATA_WIDTH = 32
    ) (// basic inputs
       input wire                      clk,
       input wire                      rst,
       
       input wire [31:0]               interrupts,
       
       output wire [WB_ADDR_WIDTH-1:0] wb_peripherals_addr,
       input wire [WB_DATA_WIDTH-1:0]  wb_peripherals_rdata,
       output wire [WB_DATA_WIDTH-1:0] wb_peripherals_wdata,
       output wire                     wb_peripherals_we,
       output wire [WB_SEL_WIDTH-1:0]  wb_peripherals_sel,
       input wire                      wb_peripherals_ack,
       output wire                     wb_peripherals_cyc,
       output wire                     wb_peripherals_stb,
       input wire                      wb_peripherals_err,

       input wire                      spi_blocked,
       output wire                     spi_busy,
       output wire                     spi_clk,
       output wire                     spi_sel,
       output wire [3:0]               spi_d_out,
       input wire [3:0]                spi_d_in,
       output wire [3:0]               spi_d_dir
    );

    localparam WB_SEL_WIDTH  = 4;
    localparam WB_FULL_ADDR_WIDTH = 32 - $clog2(4);
    localparam WB_MUX_WIDTH  = 4;

    localparam XBAR_ADDR_WIDTH = 4;
    localparam SELECTOR_ADDR_WIDTH = 2;
  
    localparam WB_XADDR_WIDTH = WB_FULL_ADDR_WIDTH - XBAR_ADDR_WIDTH;
    localparam WB_SADDR_WIDTH = WB_XADDR_WIDTH - SELECTOR_ADDR_WIDTH;


    //---------------------------------------------------------------
    // CPU

    // Instruction Bus wishbone signals (classic)
    wire [WB_FULL_ADDR_WIDTH-1:0] wbc_ibus_addr;
    wire [WB_DATA_WIDTH-1:0]      wbc_ibus_rdata;
    wire [WB_DATA_WIDTH-1:0]      wbc_ibus_wdata;
    wire                          wbc_ibus_we;
    wire [WB_SEL_WIDTH-1:0]       wbc_ibus_sel;
    wire                          wbc_ibus_ack;
    wire                          wbc_ibus_cyc;
    wire                          wbc_ibus_stb;
    wire                          wbc_ibus_err;
    wire [1:0]                    wbc_ibus_bte;
    wire [2:0]                    wbc_ibus_cti;
    
    // Data Bus wishbone signals (classic)
    wire [WB_FULL_ADDR_WIDTH-1:0] wbc_dbus_addr;
    wire [WB_DATA_WIDTH-1:0]      wbc_dbus_rdata;
    wire [WB_DATA_WIDTH-1:0]      wbc_dbus_wdata;
    wire                          wbc_dbus_we;
    wire [WB_SEL_WIDTH-1:0]       wbc_dbus_sel;
    wire                          wbc_dbus_ack;
    wire                          wbc_dbus_cyc;
    wire                          wbc_dbus_stb;
    wire                          wbc_dbus_err;
    wire [1:0]                    wbc_dbus_bte;
    wire [2:0]                    wbc_dbus_cti;

    VexRiscv vexcore(
      .clk   ( clk ),
      .reset ( rst ),
                     
      .externalResetVector    ( 32'h00000000 ),
      .timerInterrupt         ( 1'b0 ),
      .softwareInterrupt      ( 1'b0 ),
      .externalInterruptArray ( interrupts ),
  
      // Instruction Bus.
      .iBusWishbone_CYC      ( wbc_ibus_cyc   ),
      .iBusWishbone_STB      ( wbc_ibus_stb   ),
      .iBusWishbone_ACK      ( wbc_ibus_ack   ),
      .iBusWishbone_WE       ( wbc_ibus_we    ),
      .iBusWishbone_ADR      ( wbc_ibus_addr  ),
      .iBusWishbone_DAT_MISO ( wbc_ibus_rdata ),
      .iBusWishbone_DAT_MOSI ( wbc_ibus_wdata ),
      .iBusWishbone_SEL      ( wbc_ibus_sel   ),
      .iBusWishbone_ERR      ( wbc_ibus_err   ),
      .iBusWishbone_BTE      ( wbc_ibus_bte   ),
      .iBusWishbone_CTI      ( wbc_ibus_cti   ),
  
      // Data Bus.
      .dBusWishbone_CYC      ( wbc_dbus_cyc   ),
      .dBusWishbone_STB      ( wbc_dbus_stb   ),
      .dBusWishbone_ACK      ( wbc_dbus_ack   ),
      .dBusWishbone_WE       ( wbc_dbus_we    ),
      .dBusWishbone_ADR      ( wbc_dbus_addr  ),
      .dBusWishbone_DAT_MISO ( wbc_dbus_rdata ),
      .dBusWishbone_DAT_MOSI ( wbc_dbus_wdata ),
      .dBusWishbone_SEL      ( wbc_dbus_sel   ),
      .dBusWishbone_ERR      ( wbc_dbus_err   ),
      .dBusWishbone_BTE      ( wbc_dbus_bte   ),
      .dBusWishbone_CTI      ( wbc_dbus_cti   )
    );
    
    //---------------------------------------------------------------
    // CPU wishbone components
    
    // Instantiate the boot ROM.
    wire [WB_XADDR_WIDTH-1:0] wb_boot_addr;
    wire [WB_DATA_WIDTH-1:0]  wb_boot_rdata;
    wire [WB_DATA_WIDTH-1:0]  wb_boot_wdata;
    wire                      wb_boot_we;
    wire [WB_SEL_WIDTH-1:0]   wb_boot_sel;
    wire                      wb_boot_ack;
    wire                      wb_boot_cyc;
    wire                      wb_boot_err;
    wire                      wb_boot_stb;

    // Instantiate the SPRAM.
    wire [WB_XADDR_WIDTH-1:0] wb_spram1_addr;
    wire [WB_DATA_WIDTH-1:0]  wb_spram1_rdata;
    wire [WB_DATA_WIDTH-1:0]  wb_spram1_wdata;
    wire                      wb_spram1_we;
    wire [WB_SEL_WIDTH-1:0]   wb_spram1_sel;
    wire                      wb_spram1_ack;
    wire                      wb_spram1_cyc;
    wire                      wb_spram1_stb;

    // Instantiate the SPRAM.
    wire [WB_XADDR_WIDTH-1:0] wb_spram2_addr;
    wire [WB_DATA_WIDTH-1:0]  wb_spram2_rdata;
    wire [WB_DATA_WIDTH-1:0]  wb_spram2_wdata;
    wire                      wb_spram2_we;
    wire [WB_SEL_WIDTH-1:0]   wb_spram2_sel;
    wire                      wb_spram2_ack;
    wire                      wb_spram2_cyc;
    wire                      wb_spram2_stb;

    // The peripheral interface is on the module itself
    wire [WB_XADDR_WIDTH-1:0] wb_periph_real_addr;
    assign wb_peripherals_addr = wb_periph_real_addr[WB_ADDR_WIDTH-1:0];
                                     
    // Create the Wishbone crossbar.
    wbcxbar#(
      .NM(2), // One port each for instruction and data access from the CPU.
      .AW(WB_FULL_ADDR_WIDTH),
      .DW(WB_DATA_WIDTH),
      .MUXWIDTH( XBAR_ADDR_WIDTH ),
      .NS(4), // One port for SRAM, boot ROM and PWM LED driver.
      .SLAVE_MUX({
          { 2'h0 },  // Base address of the boot and data peripherals. 0x00000000
          { 2'h1 },  // Base address of the SPRAM1 (Program).          0x10000000
          { 2'h2 },  // Base address of the SPRAM2 (Data).             0x20000000
          { 2'h3 }   // Base address of the peripherals                0x30000000
      })
    ) vexcrossbar (
      .i_clk  ( clk ),
      .i_reset( rst ),
  
      // Crossbar Master Ports.
      .i_mcyc  ({ wbc_ibus_cyc,   wbc_dbus_cyc   }),
      .i_mstb  ({ wbc_ibus_stb,   wbc_dbus_cyc   }),
      .i_mwe   ({ wbc_ibus_we,    wbc_dbus_we    }),
      .i_maddr ({ wbc_ibus_addr,  wbc_dbus_addr  }),
      .i_mdata ({ wbc_ibus_wdata, wbc_dbus_wdata }),
      .i_msel  ({ wbc_ibus_sel,   wbc_dbus_sel   }),
      .o_mack  ({ wbc_ibus_ack,   wbc_dbus_ack   }),
      .o_merr  ({ wbc_ibus_err,   wbc_dbus_err   }),
      .o_mdata ({ wbc_ibus_rdata, wbc_dbus_rdata }),
  
      // Crossbar Slave Ports.
      .o_scyc  ({ wb_boot_cyc,   wb_spram1_cyc,   wb_spram2_cyc,   wb_peripherals_cyc   }),
      .o_sstb  ({ wb_boot_stb,   wb_spram1_stb,   wb_spram2_stb,   wb_peripherals_stb   }),
      .o_swe   ({ wb_boot_we,    wb_spram1_we,    wb_spram2_we,    wb_peripherals_we    }),
      .o_saddr ({ wb_boot_addr,  wb_spram1_addr,  wb_spram2_addr,  wb_periph_real_addr  }),
      .o_sdata ({ wb_boot_wdata, wb_spram1_wdata, wb_spram2_wdata, wb_peripherals_wdata }),
      .o_ssel  ({ wb_boot_sel,   wb_spram1_sel,   wb_spram2_sel,   wb_peripherals_sel   }),
      .i_sack  ({ wb_boot_ack,   wb_spram1_ack,   wb_spram2_ack,   wb_peripherals_ack   }),
      .i_serr  ({ wb_boot_err,   1'b0,            1'b0,            wb_peripherals_err   }),
      .i_sdata ({ wb_boot_rdata, wb_spram1_rdata, wb_spram2_rdata, wb_peripherals_rdata }) 
    );

    

    //---------------------------------------------------------------
    // boot and misc region router

    // Instantiate the boot ROM.
    wire [WB_SADDR_WIDTH-1:0] wb_bootrom_addr;
    wire [WB_DATA_WIDTH-1:0]  wb_bootrom_rdata;
    wire [WB_DATA_WIDTH-1:0]  wb_bootrom_wdata;
    wire                      wb_bootrom_we;
    wire [WB_SEL_WIDTH-1:0]   wb_bootrom_sel;
    wire                      wb_bootrom_ack;
    wire                      wb_bootrom_cyc;
    wire                      wb_bootrom_stb;
  
    // Instantiate the SRAM.
    wire [WB_SADDR_WIDTH-1:0] wb_sram_addr;
    wire [WB_DATA_WIDTH-1:0]  wb_sram_rdata;
    wire [WB_DATA_WIDTH-1:0]  wb_sram_wdata;
    wire                      wb_sram_we;
    wire [WB_SEL_WIDTH-1:0]   wb_sram_sel;
    wire                      wb_sram_ack;
    wire                      wb_sram_cyc;
    wire                      wb_sram_stb;

    // SPI Interface
    wire [WB_SADDR_WIDTH-1:0] wb_spi_addr;
    wire [WB_DATA_WIDTH-1:0]  wb_spi_rdata;
    wire [WB_DATA_WIDTH-1:0]  wb_spi_wdata;
    wire                      wb_spi_we;
    wire [WB_SEL_WIDTH-1:0]   wb_spi_sel;
    wire                      wb_spi_ack;
    wire                      wb_spi_cyc;
    wire                      wb_spi_stb;
    
    wbcrouter#(
      .AW       ( 28 ),
      .DW       ( WB_DATA_WIDTH ),
      .MUXWIDTH ( SELECTOR_ADDR_WIDTH ),
      .NS       ( 3 ), // Number of slaves
      .SLAVE_MUX({
          { 2'h0 },  // Base address of the boot ROM      0x00000000
          { 2'h1 },  // Base address of the SRAM (stack)  0x01000000
          { 2'h3 }   // Base address of the SPI flash     0x02000000
      })
    ) vexrouter (
      .i_clk  ( clk ),
      .i_reset( rst ),
  
      // Crossbar Master Ports.
      .i_mcyc  ( wb_boot_cyc   ),
      .i_mstb  ( wb_boot_stb   ),
      .i_mwe   ( wb_boot_we    ),
      .i_maddr ( wb_boot_addr  ),
      .i_mdata ( wb_boot_wdata ),
      .i_msel  ( wb_boot_sel   ),
      .o_mack  ( wb_boot_ack   ),
      .o_merr  ( wb_boot_err   ),
      .o_mdata ( wb_boot_rdata ),
  
      // Crossbar Slave Ports.
      .o_scyc  ({ wb_bootrom_cyc,   wb_sram_cyc,   wb_spi_cyc    }),
      .o_sstb  ({ wb_bootrom_stb,   wb_sram_stb,   wb_spi_stb    }),
      .o_swe   ({ wb_bootrom_we,    wb_sram_we,    wb_spi_we     }),
      .o_saddr ({ wb_bootrom_addr,  wb_sram_addr,  wb_spi_addr   }),
      .o_sdata ({ wb_bootrom_wdata, wb_sram_wdata, wb_spi_wdata  }),
      .o_ssel  ({ wb_bootrom_sel,   wb_sram_sel,   wb_spi_sel    }),
      .i_sack  ({ wb_bootrom_ack,   wb_sram_ack,   wb_spi_ack    }),
      .i_serr  ({ 1'b0,             1'b0,          1'b0          }),
      .i_sdata ({ wb_bootrom_rdata, wb_sram_rdata, wb_spi_rdata  })
    );

  
  
    //---------------------------------------------------------------
    // Boot Memory
    wbbootmem#(
      .AW(WB_SADDR_WIDTH),
      .DW(WB_DATA_WIDTH)
    ) vexbootmem(
      .wb_clk_i  ( clk ),
      .wb_reset_i( rst ),
      .wb_adr_i  ( wb_bootrom_addr  ),
      .wb_dat_i  ( wb_bootrom_wdata ),
      .wb_dat_o  ( wb_bootrom_rdata ),
      .wb_we_i   ( wb_bootrom_we    ),
      .wb_sel_i  ( wb_bootrom_sel   ),
      .wb_ack_o  ( wb_bootrom_ack   ),
      .wb_cyc_i  ( wb_bootrom_cyc   ),
      .wb_stb_i  ( wb_bootrom_stb   )
    );
  
    //---------------------------------------------------------------
    // SRAM
    wbsram#(
      .AW(WB_SADDR_WIDTH),
      .DW(WB_DATA_WIDTH)
    ) vexsram(
      .wb_clk_i  ( clk ),
      .wb_reset_i( rst ),
      .wb_adr_i  ( wb_sram_addr  ),
      .wb_dat_i  ( wb_sram_wdata ),
      .wb_dat_o  ( wb_sram_rdata ),
      .wb_we_i   ( wb_sram_we    ),
      .wb_sel_i  ( wb_sram_sel   ),
      .wb_ack_o  ( wb_sram_ack   ),
      .wb_cyc_i  ( wb_sram_cyc   ),
      .wb_stb_i  ( wb_sram_stb   )
    );

    //---------------------------------------------------------------
    // QSPI Flash
    wb_qspi_flash #(
      .AW( WB_SADDR_WIDTH ),
      .DW( WB_DATA_WIDTH )
    ) wb_qspi_flash_inst (
      .wb_clk_i   ( clk ),
      .wb_reset_i ( rst ),
    
      // Wishbone interface
      .wb_adr_i ( wb_spi_addr  ),
      .wb_dat_i ( wb_spi_wdata ),
      .wb_dat_o ( wb_spi_rdata ),
      .wb_we_i  ( wb_spi_we    ),
      .wb_sel_i ( wb_spi_sel   ),
      .wb_stb_i ( wb_spi_stb   ),
      .wb_cyc_i ( wb_spi_cyc   ),
      .wb_ack_o ( wb_spi_ack   ),
    
      // (Q)SPI interface
      .spi_clk   ( spi_clk   ),
      .spi_sel   ( spi_sel   ),
      .spi_d_out ( spi_d_out ),
      .spi_d_in  ( spi_d_in  ),
      .spi_d_dir ( spi_d_dir )
    );
  
    //---------------------------------------------------------------
    // SPRAM
    wbspram #(
      .AW ( WB_XADDR_WIDTH ),
      .DW ( WB_DATA_WIDTH )
    ) spram1_inst (
      // Wishbone interface.
      .wb_clk_i   ( clk ),
      .wb_reset_i ( rst ),
      .wb_adr_i   ( wb_spram1_addr  ),
      .wb_dat_i   ( wb_spram1_wdata ),
      .wb_dat_o   ( wb_spram1_rdata ),
      .wb_we_i    ( wb_spram1_we    ),
      .wb_sel_i   ( wb_spram1_sel   ),
      .wb_ack_o   ( wb_spram1_ack   ),
      .wb_cyc_i   ( wb_spram1_cyc   ),
      .wb_stb_i   ( wb_spram1_stb   )
    );

    wbspram #(
      .AW ( WB_XADDR_WIDTH ),
      .DW ( WB_DATA_WIDTH )
    ) spram2_inst (
      // Wishbone interface.
      .wb_clk_i   ( clk ),
      .wb_reset_i ( rst ),
      .wb_adr_i   ( wb_spram2_addr  ),
      .wb_dat_i   ( wb_spram2_wdata ),
      .wb_dat_o   ( wb_spram2_rdata ),
      .wb_we_i    ( wb_spram2_we    ),
      .wb_sel_i   ( wb_spram2_sel   ),
      .wb_ack_o   ( wb_spram2_ack   ),
      .wb_cyc_i   ( wb_spram2_cyc   ),
      .wb_stb_i   ( wb_spram2_stb   )
    );
  

  
endmodule
