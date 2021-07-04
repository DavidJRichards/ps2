//-----------------------------------------------------------------------------
//
// Author: John Clayton
// Date  : April 30, 2001
// Update: April 30, 2001 Copied this file from "lcd_3.v" for use as top level
// Update: May    3, 2001 Included instantiations of VHDL code, to test mixed
//                        language support of the XILINX foundation 3.1 tools.
// Update: June   1, 2001 Brigham Young's 200th birthday!
//                        Started project "build_4" using ps2 interface and
//                        the rs232 interface code.
//
// Description
//-----------------------------------------------------------------------------
// This targets an XC2S200 board which was created for educational purposes.
//
// There are:
//    8  LEDs (led[7:0])
//    4  switches (switch[3:0])
//    1  clock of 32.000 MHz clock, present on GCLK1
//    1  clock of 49.152 MHz clock, present on GCLK0
//    4  lines of ps2 clock input (port A in documentation notes)
//    4  lines of ps2 data input (port A in documentation notes)
//    16 lines of LCD panel control (port B in documentation notes)
//    2  lines of rs232 serial connection (port C in documentation notes)
//-----------------------------------------------------------------------------


`resetall
`timescale 1ns/100ps

module top (
  sys_clk,      // 33.333 MHz
  reset_n,      // push button
  switch,       // bcd switch
  led,          // 8 leds
  ps2_clk,
  ps2_data,
  rs232_rxd,
  rs232_txd
  );
  
// I/O declarations
input sys_clk;     
input reset_n;

input [3:0] switch;
output [7:0] led;

input rs232_rxd;
output rs232_txd;

inout ps2_clk;
inout ps2_data;

// Internal signal declarations

wire [7:0] ps2_scan_code;
wire [7:0] ps2_ascii;
wire [7:0] ps2_status;
wire [7:0] rs232_rx_character;

wire caps_lock;
wire num_lock;
wire scroll_lock;
wire shift_key_on;
wire crl_key_on;
wire update_leds;

wire reset;// = ~reset_n | ~pup_reset;
wire [2:0] rs232_rx_error;
wire ps2_key_data_ready;
wire ps2_key_extended;
wire ps2_key_released;
wire ps2_key_pressed = ~ps2_key_released;
wire rx232_tx_load_request;
wire rs232_rx_data_ready;

reg [7:0] tx_state;
reg [7:0] tx_next_state;

parameter tx_reset = 1;
parameter tx_reset_send = 2;
parameter tx_reset_strb = 3;
parameter tx_idle  = 4;
parameter tx_byte1 = 5;
parameter tx_strb1 = 6;
parameter tx_wack1 = 8;
parameter tx_wait_ack = 9;
parameter tx_byte2 = 16;
parameter tx_strb2 = 32;
parameter tx_wack2 = 64;
parameter tx_done = 128;

reg [7:0] tx_data;
reg tx_write;
wire tx_write_ack;
wire rx_acknowledge;

wire [7:0] esc_ascii;
wire esc_active;
wire esc_flag;
//--------------------------------------------------------------------------
// Instantiations
//--------------------------------------------------------------------------
`define ALT_TX_DATA // select if data sent to keyboard is keyboard LED status (normal) or received serial data (testing)

ps2_keyboard_interface #(2950, // number of clks for 60usec.
                         12,   // number of bits needed for 60usec. timer
                         63,   // number of clks for debounce
                         6,    // number of bits needed for debounce timer
                         1     // Trap the shift keys, no event generated
                         )                       
  ps2_block (                  // Instance name
  .clk(sys_clk),
  .reset(reset),
  .ps2_clk(ps2_clk),
  .ps2_data(ps2_data),
  .rx_extended(ps2_key_extended),
  .rx_released(ps2_key_released),
  .rx_acknowledge(rx_acknowledge),
  .caps_lock(caps_lock),
  .num_lock(num_lock),
  .scroll_lock(scroll_lock),
  .update_leds(update_leds),
  .rx_shift_key_on(shift_key_on),
  .rx_ctrl_key_on(ctrl_key_on),
  .rx_scan_code(ps2_scan_code),
  .rx_ascii(ps2_ascii),
  .rx_data_ready(ps2_key_data_ready),
  .rx_read(ps2_key_data_ready),

  .esc_ascii(esc_ascii),
  .esc_active(esc_active),
  .esc_flag(esc_flag),  
  
`ifndef ALT_TX_DATA
  .tx_data(rs232_rx_character),
  .tx_write(rs232_rx_data_ready),
  .tx_write_ack_o(ps2_tx_write_ack_o),
`else
  .tx_data(tx_data),
  .tx_write(tx_write),
  .tx_write_ack_o(tx_write_ack),
`endif  
  .tx_error_no_keyboard_ack(ps2_status[4])
  );
assign ps2_status[6] = ps2_key_released;
assign ps2_status[3] = rs232_rx_data_ready;

// These defines are for the rs232 interface
`define START_BITS 1
`define DATA_BITS 8
`define STOP_BITS 1
`define CLOCK_FACTOR 16

// This unit generates the correct 16x transmit clock (enable) frequency
// which is used for the serial transmit operation.
clock_gen_select clock_unit
  (
   .clk(sys_clk),
   .reset(reset),
   .rate_select(3'b100),         // not 115,200 baud
   .clk_out(serial_clk_16x)
  );

// A transmitter, which asserts load_request at the end of the currently
// transmitted word.  The tx_clk must be a "clock enable" (narrow positive
// pulse) which occurs at 16x the desired transmit rate.  If load_request
// is connected directly to load, the unit will transmit continuously.
rs232tx #(
          `START_BITS,   // start_bits
          `DATA_BITS,    // data_bits
          `STOP_BITS,    // stop_bits (add intercharacter delay...)
          `CLOCK_FACTOR  // clock_factor
         )
         rs232_tx_block // instance name
         ( 
          .clk(sys_clk),
          .tx_clk(serial_clk_16x),
          .reset(reset),
          .load(   ps2_key_data_ready
                && (ps2_key_pressed || esc_flag)
                && rs232_tx_load_request),
          .data(ps2_ascii),                 // Connected from keyboard
          .load_request(rs232_tx_load_request),
          .txd(rs232_txd)
         );

// A receiver, which asserts "word_ready" to indicate a received word.
// Asserting "read_word" will cause "word_ready" to go low again if it was high.
// The character is held in the output register, during the time the next
//   character is coming in.
rs232rx #(
          `START_BITS,  // start_bits
          `DATA_BITS,   // data_bits
          `STOP_BITS,   // stop_bits
          `CLOCK_FACTOR // clock_factor
         )
         rs232_rx_block // instance name
         ( 
          .clk(sys_clk),
          .rx_clk(serial_clk_16x),
          .reset(reset || (| rs232_rx_error) ),
          .rxd(rs232_rxd),
          .read(ps2_tx_write_ack_o),
          .data(rs232_rx_character),
          .data_ready(rs232_rx_data_ready),
          .error_over_run(rs232_rx_error[0]),
          .error_under_run(rs232_rx_error[1]),
          .error_all_low(rs232_rx_error[2])
         );

//--------------------------------------------------------------------------
// Module code
//--------------------------------------------------------------------------
    // PowerUP Reset Logic
    // generate a 500ms reset pulse on initial powerup
    reg         [25:0] pup_count;
    reg         pup_reset = 1'b0;
    
    always @(posedge sys_clk)
        begin
        if(~pup_reset) 
            pup_count <= pup_count + 1;
        if (pup_count >= 16000000) 
            pup_reset <= 1'b1;
        end
        
    assign reset = ~reset_n || ~pup_reset;


//---------------------
// drive keyboard LEDs
// send power on reset
//---------------------

// State register
always @(posedge sys_clk, posedge reset)
begin : tx_state_register
  if (reset)
    tx_state <= tx_reset;
  else
     tx_state <= tx_next_state;
 end

// State transition logic
always @(tx_state, update_leds, tx_write_ack, rx_acknowledge, scroll_lock, num_lock, caps_lock
         )
begin      

    tx_next_state = tx_state;
  
    case (tx_state)

    tx_reset :
        begin
        tx_write <= 0;
        tx_next_state <= tx_reset_send;
        end

    tx_reset_send :
        begin
        tx_write <= 1;
        tx_data <= 8'hFF;
        tx_next_state <= tx_reset_strb;
        end

    tx_reset_strb :
        begin
        tx_next_state <= tx_idle;
        end

    tx_idle : 
        begin
        tx_write <= 0;
        if (update_leds )
            tx_next_state <= tx_byte1;
        end 
//----------------------------------------------
    tx_byte1 :
        begin
            tx_data <= 8'hED;
            tx_write <= 1;
            tx_next_state <= tx_strb1;
        end

    tx_strb1:
        begin
        if(tx_write_ack)
            tx_next_state <= tx_byte2;
        else
            tx_next_state <= tx_wack1;
        end 
        
    tx_wack1 :
        begin
        tx_write <= 0;
        tx_next_state <= tx_wait_ack;
        end 

    tx_wait_ack :
        if(rx_acknowledge)
            tx_next_state <= tx_byte2;
    
//------------------------------------------------          
    tx_byte2 :
        begin
            tx_data <= {5'h00, caps_lock, num_lock, scroll_lock};
            tx_write <= 1;
            tx_next_state <= tx_strb2;
         end

    tx_strb2 :
        begin
        if(tx_write_ack)
            tx_next_state <= tx_done;
        else
            tx_next_state <= tx_wack2;
        
        end 
               
    tx_wack2: 
        begin
        tx_write <= 0;
        tx_next_state <= tx_done;
        end 

    tx_done:
        tx_next_state <= tx_idle;
                    
    endcase
end

//------------------------------------------------
// debug input and output using LEDs and switches
//------------------------------------------------
assign led[7:0] = fn(~switch[3:0]);

function [7:0]fn(input [3:0]code);
    case (code)
        0: fn = ps2_scan_code;
        1: fn = rs232_rx_character;
        2: fn = ps2_status;
        3: fn = {reset, ~reset_n, ~pup_reset, ~ps2_clk,~ps2_data};
        4: fn = esc_ascii;
        5: fn = {ps2_key_released,ps2_key_pressed,esc_active,esc_flag};
        6: fn = ps2_ascii;
        7: fn = {ps2_key_released, ps2_key_extended, ps2_key_pressed, ps2_key_data_ready};
        8: fn = {ctrl_key_on, shift_key_on, update_leds, caps_lock, num_lock, scroll_lock};
        9: fn = {rs232_rx_error};
        default: fn = 8'h55;
    endcase
endfunction

endmodule
