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

module top (
        input wire   pin_clk,
     
        inout wire   pin_usbp,
        inout wire   pin_usbn,
        output logic pin_pu,
        
        inout wire   pin_miso,
        inout wire   pin_mosi,
        inout wire   pin_wp,
        inout wire   pin_hold,
        output logic pin_cs,
        output logic pin_sck,

        input wire   pin_button,
     
        output logic pin_stat_r,
        output logic pin_stat_g,
        output logic pin_stat_b,

        // USB-PD interface
        output logic pin_cc_vpwm,
        input wire   pin_cc_va,
        input wire   pin_cc_vb,
        output logic pin_cc_dir,
        output logic pin_cc_a,
        output logic pin_cc_b
    );


    wire          stat_r;
    wire          stat_g;
    wire          stat_b;
    wire          stat_en;
  
    SB_RGBA_DRV #(
        .CURRENT_MODE ( "0b1"        ), // half current mode
        .RGB0_CURRENT ( "0b00000001" ), // 4mA
        .RGB1_CURRENT ( "0b00000001" ),
        .RGB2_CURRENT ( "0b00000001" )
    ) rgb_drv_inst (
        .RGBLEDEN ( stat_en    ),
        .CURREN   ( stat_en    ),
        .RGB0PWM  ( stat_b     ),
        .RGB1PWM  ( stat_r     ),
        .RGB2PWM  ( stat_g     ),
        .RGB0     ( pin_stat_r ),
        .RGB1     ( pin_stat_g ),
        .RGB2     ( pin_stat_b )
    );
    assign stat_en = 1;
  
    wire [15:0]   debug;
  
    wire flash_busy;
    assign flash_busy = 0;
    
    wire serial_irq;
      
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////
    //////// Generate Clocks
    ////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    wire clk_96mhz;
    wire clk_locked;
    pll96mhz pll(
      .refclk     ( pin_clk    ),
      .clk_output ( clk_96mhz  ),
      .clk_locked ( clk_locked )
    );

    // Clock Division - produce a 48Mhz and 16Mhz clock.
    reg clk_48mhz = 0;
    reg clk_16mhz = 0;
    always @(posedge clk_96mhz) clk_48mhz = ~clk_48mhz;
    reg [1:0] clk_divide = 0;
    always @(posedge clk_96mhz) begin
      if (clk_divide) clk_divide <= clk_divide - 1;
      else begin
        clk_divide <= 2;
        clk_16mhz <= ~clk_16mhz;
      end
    end
  
    wire lf_clk;
    SB_LFOSC LF_OscInst (
      .CLKLFPU (1),
      .CLKLFEN (1),
      .CLKLF   (lf_clk)
    );
  
    // Generate reset signal
    reg [3:0]     reset_cnt = 7;
    reg           reset;
    
    always @(posedge lf_clk) begin
      if (reset_cnt) begin
        reset     <= 1;
        reset_cnt <= reset_cnt - 1;
      end
      else begin
        reset     <= 0;
        reset_cnt <= 0;
      end
    end

    wire clk;
    wire rst;
    localparam CLK_FREQ = 16000000;
    assign clk = pin_clk;//clk_16mhz;
    assign rst = reset;

    //============================================================
    // interrupt connections
    logic [31:0] interrupts;
    always_comb begin
        interrupts[0]    = button_irq;
        interrupts[1]    = serial_irq;
        interrupts[31:2] = '0;
    end

    //============================================================
    // base system
    localparam BASE_ADDR_WIDTH = 20;
    localparam SELECTOR_ADDR_WIDTH = 4;
    localparam WB_DATA_WIDTH = 32;
    localparam WB_SEL_WIDTH  = (WB_DATA_WIDTH / 8);
  
    localparam WB_ADDR_WIDTH = BASE_ADDR_WIDTH - SELECTOR_ADDR_WIDTH;
    
    wire [BASE_ADDR_WIDTH-1:0] wb_peripherals_addr;
    wire [WB_DATA_WIDTH-1:0]   wb_peripherals_rdata;
    wire [WB_DATA_WIDTH-1:0]   wb_peripherals_wdata;
    wire                       wb_peripherals_we;
    wire [WB_SEL_WIDTH-1:0]    wb_peripherals_sel;
    wire                       wb_peripherals_ack;
    wire                       wb_peripherals_cyc;
    wire                       wb_peripherals_stb;
    wire                       wb_peripherals_err;

    wire                       spi_blocked;
    wire                       spi_busy;
    wire                       spi_clk;
    wire                       spi_sel;
    wire [3:0]                 spi_d_out;
    wire [3:0]                 spi_d_in;
    wire [3:0]                 spi_d_dir;
    
    base_system 
     #(
       .WB_ADDR_WIDTH ( BASE_ADDR_WIDTH ),
       .CLK_FREQ      ( 16000000 )
    ) base_system_inst ( // basic inputs
       .clk ( clk ),
       .rst ( rst ),
       
       .interrupts ( interrupts ),
       
       .wb_peripherals_addr  ( wb_peripherals_addr  ),
       .wb_peripherals_rdata ( wb_peripherals_rdata ),
       .wb_peripherals_wdata ( wb_peripherals_wdata ),
       .wb_peripherals_we    ( wb_peripherals_we    ),
       .wb_peripherals_sel   ( wb_peripherals_sel   ),
       .wb_peripherals_ack   ( wb_peripherals_ack   ),
       .wb_peripherals_cyc   ( wb_peripherals_cyc   ),
       .wb_peripherals_stb   ( wb_peripherals_stb   ),
       .wb_peripherals_err   ( wb_peripherals_err   ),

       .spi_blocked ( spi_blocked ),
       .spi_busy    ( spi_busy    ),
       .spi_clk     ( spi_clk     ),
       .spi_sel     ( spi_sel     ),
       .spi_d_out   ( spi_d_out   ),
       .spi_d_in    ( spi_d_in    ),
       .spi_d_dir   ( spi_d_dir   )
    );

  
    
    //---------------------------------------------------------------
    wire [WB_ADDR_WIDTH-1:0] wb_serial_addr;
    wire [WB_DATA_WIDTH-1:0] wb_serial_rdata;
    wire [WB_DATA_WIDTH-1:0] wb_serial_wdata;
    wire                     wb_serial_we;
    wire [WB_SEL_WIDTH-1:0]  wb_serial_sel;
    wire                     wb_serial_ack;
    wire                     wb_serial_cyc;
    wire                     wb_serial_stb;
    
    // Wishbone connected LED driver.
    wire [WB_ADDR_WIDTH-1:0] wb_misc_addr;
    wire [WB_DATA_WIDTH-1:0] wb_misc_rdata;
    wire [WB_DATA_WIDTH-1:0] wb_misc_wdata;
    wire                     wb_misc_we;
    wire [WB_SEL_WIDTH-1:0]  wb_misc_sel;
    wire                     wb_misc_ack;
    wire                     wb_misc_cyc;
    wire                     wb_misc_stb;
  
    wbcrouter#(
      .AW       ( BASE_ADDR_WIDTH ),
      .DW       ( WB_DATA_WIDTH ),
      .MUXWIDTH ( SELECTOR_ADDR_WIDTH ),
      .NS       ( 2 ), // Number of slaves
      .SLAVE_MUX({
          { 4'h0 },  // Base address of the misc registers        0x30000000
          { 4'h1 }   // Base address of the USB Serial interface. 0x30010000
      })
    ) vexrouter (
      .i_clk  ( clk ),
      .i_reset( rst ),
  
      // Crossbar Master Ports.
      .i_mcyc  ( wb_peripherals_cyc   ),
      .i_mstb  ( wb_peripherals_stb   ),
      .i_mwe   ( wb_peripherals_we    ),
      .i_maddr ( wb_peripherals_addr  ),
      .i_mdata ( wb_peripherals_wdata ),
      .i_msel  ( wb_peripherals_sel   ),
      .o_mack  ( wb_peripherals_ack   ),
      .o_merr  ( wb_peripherals_err   ),
      .o_mdata ( wb_peripherals_rdata ),
  
      // Crossbar Slave Ports.
      .o_scyc  ({ wb_misc_cyc,   wb_serial_cyc    }),
      .o_sstb  ({ wb_misc_stb,   wb_serial_stb    }),
      .o_swe   ({ wb_misc_we,    wb_serial_we     }),
      .o_saddr ({ wb_misc_addr,  wb_serial_addr   }),
      .o_sdata ({ wb_misc_wdata, wb_serial_wdata  }),
      .o_ssel  ({ wb_misc_sel,   wb_serial_sel    }),
      .i_sack  ({ wb_misc_ack,   wb_serial_ack    }),
      .i_serr  ({ 1'b0,          1'b0             }),
      .i_sdata ({ wb_misc_rdata, wb_serial_rdata  })
    );

  
    //---------------------------------------------------------------
    // uart and protocol
    
    wire usb_p_tx;
    wire usb_n_tx;
    wire usb_p_rx;
    wire usb_n_rx;
    wire usb_tx_en;
    
    wire dfu_detach;
  
    // USB Serial Core.
    wb_usb_serial#(
      .AW( WB_ADDR_WIDTH ),
      .DW(WB_DATA_WIDTH)
    ) usb_serial(
      .wb_clk_i  (clk),
      .wb_reset_i(rst),
    
      // Wishbone bus.
      .wb_adr_i  (wb_serial_addr),
      .wb_dat_i  (wb_serial_wdata),
      .wb_dat_o  (wb_serial_rdata),
      .wb_we_i   (wb_serial_we),
      .wb_sel_i  (wb_serial_sel),
      .wb_ack_o  (wb_serial_ack),
      .wb_cyc_i  (wb_serial_cyc),
      .wb_stb_i  (wb_serial_stb),
    
      // USB lines.
      .usb_clk   (clk_48mhz),
      .usb_p_tx  (usb_p_tx),
      .usb_n_tx  (usb_n_tx),
      .usb_p_rx  (usb_p_rx),
      .usb_n_rx  (usb_n_rx),
      .usb_tx_en (usb_tx_en),
      
      // DFU state and debug
      .irq(serial_irq),
      .dfu_detach(dfu_detach),
      .debug()
    );
    usb_phy_ice40 usb_phy(
      .pin_usb_p (pin_usbp),
      .pin_usb_n (pin_usbn),
    
      .usb_p_tx  (usb_p_tx),
      .usb_n_tx  (usb_n_tx),
      .usb_p_rx  (usb_p_rx),
      .usb_n_rx  (usb_n_rx),
      .usb_tx_en (usb_tx_en)
    );
    assign pin_pu = 1'b1;
  
  
    //---------------------------------------------------------------
    // wishbone connected LED PWM driver
    wire [1:0] buttons;
    wire       button_irq;

    assign buttons = { pin_button };
    
    wb_misc #(
      .AW( WB_ADDR_WIDTH ),
      .DW( WB_DATA_WIDTH )
    ) wb_misc_inst (
      .wb_clk_i   ( clk ),
      .wb_reset_i ( rst ),
      .wb_adr_i   ( wb_misc_addr ),
      .wb_dat_i   ( wb_misc_wdata ),
      .wb_dat_o   ( wb_misc_rdata ),
      .wb_we_i    ( wb_misc_we ),
      .wb_sel_i   ( wb_misc_sel ),
      .wb_ack_o   ( wb_misc_ack ),
      .wb_cyc_i   ( wb_misc_cyc ),
      .wb_stb_i   ( wb_misc_stb ),
  
      .leds       ( { stat_b, stat_g, stat_r } ),
      .buttons    ( buttons ),
      .irq        ( button_irq )
    );

    //logic [31:0] led_counter;
    //initial led_counter = 0;
    //always @(posedge clk) begin
    //    if (led_counter) led_counter <= led_counter - 1;
    //    else             led_counter <= 16000000;
    //end
    //always_comb begin
    //    stat_b = led_counter < 10000;
    //    stat_g = 0;
    //    stat_r = led_counter < 10000;
    //end
    
    
  
  
    //---------------------------------------------------------------
    // qspi
    
    assign pin_sck = spi_clk;
    assign pin_cs  = spi_sel;
    
    SB_IO #(
      .PIN_TYPE( 6'b1010_01 ), // PIN_OUTPUT_TRISTATE - PIN_INPUT
      .PULLUP  ( 1'b0       )
    ) iobuf_d0 (
      .PACKAGE_PIN   ( pin_mosi     ),
      .OUTPUT_ENABLE ( spi_d_dir[0] ),
      .D_OUT_0       ( spi_d_out[0] ),
      .D_IN_0        ( spi_d_in[0]  )
    );
    SB_IO #(
      .PIN_TYPE( 6'b1010_01 ), // PIN_OUTPUT_TRISTATE - PIN_INPUT
      .PULLUP  ( 1'b0       )
    ) iobuf_d1 (
      .PACKAGE_PIN   ( pin_miso     ),
      .OUTPUT_ENABLE ( spi_d_dir[1] ),
      .D_OUT_0       ( spi_d_out[1] ),
      .D_IN_0        ( spi_d_in[1]  )
    );
    SB_IO #(
      .PIN_TYPE( 6'b1010_01 ), // PIN_OUTPUT_TRISTATE - PIN_INPUT
      .PULLUP  ( 1'b1       )
    ) iobuf_d2 (
      .PACKAGE_PIN   ( pin_wp       ),
      .OUTPUT_ENABLE ( spi_d_dir[2] ),
      .D_OUT_0       ( spi_d_out[2] ),
      .D_IN_0        ( spi_d_in[2]  )
    );
    SB_IO #(
      .PIN_TYPE( 6'b1010_01 ), // PIN_OUTPUT_TRISTATE - PIN_INPUT
      .PULLUP  ( 1'b1       )
    ) iobuf_d3 (
      .PACKAGE_PIN   ( pin_hold     ),
      .OUTPUT_ENABLE ( spi_d_dir[3] ),
      .D_OUT_0       ( spi_d_out[3] ),
      .D_IN_0        ( spi_d_in[3]  )
    );
    

                     
    
  
    // Image Slot 0: Multiboot header and POR springboard.
    // Image Slot 1: DFU Bootloader
    // Image Slot 2: This Image (User Application).
    SB_WARMBOOT warmboot_inst (
      .S1(1'b0),
      .S0(1'b1),
      .BOOT(dfu_detach)
    );
  


  
endmodule
