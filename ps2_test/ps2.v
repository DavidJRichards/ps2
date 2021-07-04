//-------------------------------------------------------------------------------------
//
// Author: John Clayton
// Date  : April 30, 2001
// Update: 4/30/01 copied this file from lcd_2.v (pared down).
// Update: 5/24/01 changed the first module from "ps2_keyboard_receiver"
//                 to "ps2_keyboard_interface"
// Update: 5/29/01 Added input synchronizing flip-flops.  Changed state
//                 encoding (m1) for good operation after part config.
// Update: 5/31/01 Added low drive strength and slow transitions to ps2_clk
//                 and ps2_data in the constraints file.  Added the signal
//                 "tx_shifting_done" as distinguished from "rx_shifting_done."
//                 Debugged the transmitter portion in the lab.
// Update: 6/01/01 Added horizontal tab to the ascii output.
// Update: 6/01/01 Added parameter TRAP_SHIFT_KEYS.
//
//
//
//
//
// Description
//-------------------------------------------------------------------------------------
// This is a state-machine driven serial-to-parallel and parallel-to-serial
// interface to the ps2 style keyboard interface.  The details of the operation
// of the keyboard interface were obtained from the following website:
//
//   http://www.beyondlogic.org/keyboard/keybrd.htm
//
// Some aspects of the keyboard interface are not implemented (e.g, parity
// checking for the receive side, and recognition of the various commands
// which the keyboard sends out, such as "power on selt test passed," "Error"
// and "Resend.")  However, if the user wishes to recognize these reply
// messages, the scan code output can always be used to extend functionality
// as desired.
//
// Note that the "Extended" (0xE0) and "Released" (0xF0) codes are recognized.
// The rx interface provides separate indicator flags for these two conditions
// with every valid character scan code which it provides.  The shift keys are
// also trapped by the interface, in order to provide correct uppercase ASCII
// characters at the ascii output, although the scan codes for the shift keys
// are still provided at the scan code output.  So, the left/right ALT keys
// can be differentiated by the presence of the rx_entended signal, while the
// left/right shift keys are differentiable by the different scan codes
// received.
//
// The interface to the ps2 keyboard uses ps2_clk clock rates of
// 30-40 kHz, dependent upon the keyboard itself.  The rate at which the state
// machine runs should be at least twice the rate of the ps2_clk, so that the
// states can accurately follow the clock signal itself.  Four times 
// oversampling is better.  Say 200kHz at least.  The upper limit for clocking
// the state machine will undoubtedly be determined by delays in the logic 
// which decodes the scan codes into ASCII equivalents.  The maximum speed
// will be most likely many megahertz, depending upon target technology.
// In order to run the state machine extremely fast, synchronizing flip-flops
// have been added to the ps2_clk and ps2_data inputs of the state machine.
// This avoids poor performance related to slow transitions of the inputs.
// 
// Because this is a bi-directional interface, while reading from the keyboard
// the ps2_clk and ps2_data lines are used as inputs.  While writing to the
// keyboard, however (which may be done at any time.  If writing interrupts a
// read from the keyboard, the keyboard will buffer up its data, and send
// it later) both the ps2_clk and ps2_data lines are occasionally pulled low,
// and pullup resistors are used to bring the lines high again, by setting
// the drivers to high impedance state.
//
// The tx interface, for writing to the keyboard, does not provide any special
// pre-processing.  It simply transmits the 8-bit command value to the
// keyboard.
//
// Pullups MUST BE USED on the ps2_clk and ps2_data lines for this design,
// whether they be internal to an FPGA I/O pad, or externally placed.
// If internal pullups are used, they may be fairly weak, causing bounces
// due to crosstalk, etc.  There is a "debounce timer" implemented in order
// to eliminate erroneous state transitions which would occur based on bounce.
// 
// Parameters are provided in order to configure and appropriately size the
// counter of a 60 microsecond timer used in the transmitter, depending on
// the clock frequency used.  The 60 microsecond period is guaranteed to be
// more than one period of the ps2_clk_s signal.
//
// Also, a smaller 5 microsecond timer has been included for "debounce".
// This is used because, with internal pullups on the ps2_clk and ps2_data
// lines, there is some bouncing around which occurs
//
// A parameter TRAP_SHIFT_KEYS allows the user to eliminate shift keypresses
// from producing scan codes (along with their "undefined" ASCII equivalents)
// at the output of the interface.  If TRAP_SHIFT_KEYS is non-zero, the shift
// key status will only be reported by rx_shift_key_on.  No ascii or scan
// codes will be reported for the shift keys.  This is useful for those who
// wish to use the ASCII data stream, and who don't want to have to "filter
// out" the shift key codes.
//
// https://web.archive.org/web/20080129193908/http://www.beyondlogic.org/keyboard/keybrd.htm
// https://techdocs.altium.com/display/FPGA/PS2+Keyboard+Scan+Codes
// https://verilogguide.readthedocs.io/en/latest/verilog/fsm.html


//-------------------------------------------------------------------------------------


`resetall
`timescale 1ns/100ps

`define TOTAL_BITS          11
`define EXTEND_CODE         16'hE0
`define RELEASE_CODE        16'hF0
`define ACKNOWLEDGE_CODE    16'hFA
`define LEFT_SHIFT          16'h12
`define RIGHT_SHIFT         16'h59
`define CAPS_CODE           16'h58
`define NUM_CODE            16'h77
`define SCROLL_CODE         16'h7E
`define LEFT_CTRL           16'h14
`define RIGHT_CTRL          16'h14


module ps2_keyboard_interface (
  clk,
  reset,
  ps2_clk,
  ps2_data,
  rx_extended,
  rx_released,
  rx_acknowledge,
  caps_lock,
  num_lock,
  scroll_lock,
  update_leds,
  rx_ctrl_key_on,
  rx_shift_key_on,
  rx_scan_code,
  rx_ascii,
  esc_ascii,
  esc_active,
  esc_flag,
  rx_data_ready,       // rx_read_o
  rx_read,             // rx_read_ack_i
  tx_data,          // tx data, 8 bits
  tx_write,         // tx strobe, set to 1 to send
  tx_write_ack_o,   // tx data sent
  tx_error_no_keyboard_ack
  );

// Parameters

// The timer value can be up to (2^bits) inclusive.
//parameter TIMER_60USEC_VALUE_PP = 1920; // Number of sys_clks for 60usec.
parameter TIMER_60USEC_VALUE_PP = 1843; // Number of sys_clks for 60usec.
parameter TIMER_60USEC_BITS_PP  = 11;   // Number of bits needed for timer
//parameter TIMER_5USEC_VALUE_PP = 186;   // Number of sys_clks for debounce
parameter TIMER_5USEC_VALUE_PP = 178;   // Number of sys_clks for debounce
parameter TIMER_5USEC_BITS_PP  = 8;     // Number of bits needed for timer
parameter TRAP_SHIFT_KEYS_PP = 0;       // Default: No shift key trap.

// State encodings, provided as parameters
// for flexibility to the one instantiating the module.
// In general, the default values need not be changed.

// State "m1_rx_clk_l" has been chosen on purpose.  Since the input
// synchronizing flip-flops initially contain zero, it takes one clk
// for them to update to reflect the actual (idle = high) status of
// the I/O lines from the keyboard.  Therefore, choosing 0 for m1_rx_clk_l
// allows the state machine to transition to m1_rx_clk_h when the true
// values of the input signals become present at the outputs of the
// synchronizing flip-flops.  This initial transition is harmless, and it
// eliminates the need for a "reset" pulse before the interface can operate.

parameter m1_rx_clk_h = 1;
parameter m1_rx_clk_l = 0;
parameter m1_rx_falling_edge_marker = 13;
parameter m1_rx_rising_edge_marker = 14;
parameter m1_tx_force_clk_l = 3;
parameter m1_tx_first_wait_clk_h = 10;
parameter m1_tx_first_wait_clk_l = 11;
parameter m1_tx_reset_timer = 12;
parameter m1_tx_wait_clk_h = 2;
parameter m1_tx_clk_h = 4;
parameter m1_tx_clk_l = 5;
parameter m1_tx_wait_keyboard_ack = 6;
parameter m1_tx_done_recovery = 7;
parameter m1_tx_error_no_keyboard_ack = 8;
parameter m1_tx_rising_edge_marker = 9;

parameter m2_rx_data_escaped_done   = 5;
parameter m2_rx_data_escaped_ack    = 4;
parameter m2_rx_data_escaped        = 3;
parameter m2_rx_data_ready          = 2;
parameter m2_rx_data_ready_ack      = 1;

  
// I/O declarations
input clk;
input reset;
inout ps2_clk;
inout ps2_data;
output rx_extended;
output rx_released;
output rx_acknowledge;
output caps_lock;
output num_lock;
output scroll_lock;
output update_leds;
output rx_shift_key_on;
output rx_ctrl_key_on;
output [7:0] rx_scan_code;
output [7:0] rx_ascii;
output [7:0] esc_ascii;
output esc_active;
output esc_flag;
output rx_data_ready;
input rx_read;
input [7:0] tx_data;
input tx_write;
output tx_write_ack_o;
output tx_error_no_keyboard_ack;

reg rx_extended;
reg rx_released;
reg rx_acknowledge;
wire caps_lock;
wire num_lock;
wire scroll_lock;
reg [7:0] rx_scan_code;
reg [7:0] rx_ascii;
//reg [7:0] esc_data;
reg rx_data_ready;
reg tx_error_no_keyboard_ack;

// Internal signal declarations
wire timer_60usec_done;
wire timer_5usec_done;
wire extended;
wire released;
//wire acknowledge;
wire shift_key_on;
wire ctrl_key_on;

                         // NOTE: These two signals used to be one.  They
                         //       were split into two signals because of
                         //       shift key trapping.  With shift key
                         //       trapping, no event is generated externally,
                         //       but the "hold" data must still be cleared
                         //       anyway regardless, in preparation for the
                         //       next scan codes.
wire rx_output_event;    // Used only to clear: hold_released, hold_extended
wire rx_output_strobe;   // Used to produce the actual output.

wire tx_parity_bit;
wire rx_shifting_done;
wire tx_shifting_done;
wire [11:0] shift_key_plus_code;

reg [`TOTAL_BITS-1:0] q;
reg [3:0] m1_state;
reg [3:0] m1_next_state;
reg [3:0] m2_state;
reg [3:0] m2_next_state;
reg [3:0] bit_count;
reg enable_timer_60usec;
reg enable_timer_5usec;
reg [TIMER_60USEC_BITS_PP-1:0] timer_60usec_count;
reg [7:0] timer_5usec_count;
reg [7:0] ascii;      // "REG" type only because a case statement is used.
reg [7:0] esc_ascii;
reg esc_active;
reg esc_flag;
reg esc_reset;
reg left_shift_key;
reg right_shift_key;
reg left_ctrl_key;
reg right_ctrl_key;
reg hold_extended;    // Holds prior value, cleared at rx_output_strobe
reg hold_released;    // Holds prior value, cleared at rx_output_strobe
//reg hold_acknowledge;
reg ps2_clk_s;        // Synchronous version of this input
reg ps2_data_s;       // Synchronous version of this input
reg ps2_clk_hi_z;     // Without keyboard, high Z equals 1 due to pullups.
reg ps2_data_hi_z;    // Without keyboard, high Z equals 1 due to pullups.

reg caps_state;
reg num_state;
reg scroll_state;
reg update_leds;

//--------------------------------------------------------------------------
// Module code

assign ps2_clk = ps2_clk_hi_z?1'bZ:1'b0;
assign ps2_data = ps2_data_hi_z?1'bZ:1'b0;

// Input "synchronizing" logic -- synchronizes the inputs to the state
// machine clock, thus avoiding errors related to
// spurious state machine transitions.
//
// Since the initial state of registers is zero, and the idle state
// of the ps2_clk and ps2_data lines is "1" (due to pullups), the
// "sense" of the ps2_clk_s signal is inverted from the true signal.
// This allows the state machine to "come up" in the correct
always @(posedge clk)
begin
  ps2_clk_s <= ps2_clk;
  ps2_data_s <= ps2_data;
end

// State register
always @(posedge clk)
begin : m1_state_register
  if (reset) m1_state <= m1_rx_clk_h;
  else m1_state <= m1_next_state;
end

// State transition logic
always @(m1_state
         or q
         or tx_shifting_done
         or tx_write
         or ps2_clk_s
         or ps2_data_s
         or timer_60usec_done
         or timer_5usec_done
         )
begin : m1_state_logic

  // Output signals default to this value, unless changed in a state condition.
  ps2_clk_hi_z <= 1;
  ps2_data_hi_z <= 1;
  tx_error_no_keyboard_ack <= 0;
  enable_timer_60usec <= 0;
  enable_timer_5usec <= 0;

  case (m1_state)

    m1_rx_clk_h :
      begin
        enable_timer_60usec <= 1;
        if (tx_write) m1_next_state <= m1_tx_reset_timer;
        else if (~ps2_clk_s) m1_next_state <= m1_rx_falling_edge_marker;
        else m1_next_state <= m1_rx_clk_h;
      end
      
    m1_rx_falling_edge_marker :
      begin
        enable_timer_60usec <= 0;
        m1_next_state <= m1_rx_clk_l;
      end

    m1_rx_rising_edge_marker :
      begin
        enable_timer_60usec <= 0;
        m1_next_state <= m1_rx_clk_h;
      end


    m1_rx_clk_l :
      begin
        enable_timer_60usec <= 1;
        if (tx_write) m1_next_state <= m1_tx_reset_timer;
        else if (ps2_clk_s) m1_next_state <= m1_rx_rising_edge_marker;
        else m1_next_state <= m1_rx_clk_l;
      end

    m1_tx_reset_timer:
      begin
        enable_timer_60usec <= 0;
        m1_next_state <= m1_tx_force_clk_l;
      end

    m1_tx_force_clk_l :
      begin
        enable_timer_60usec <= 1;
        ps2_clk_hi_z <= 0;  // Force the ps2_clk line low.
        if (timer_60usec_done) m1_next_state <= m1_tx_first_wait_clk_h;
        else m1_next_state <= m1_tx_force_clk_l;
      end

    m1_tx_first_wait_clk_h :
      begin
        enable_timer_5usec <= 1;
        ps2_data_hi_z <= 0;        // Start bit.
        if (~ps2_clk_s && timer_5usec_done)
          m1_next_state <= m1_tx_clk_l;
        else
          m1_next_state <= m1_tx_first_wait_clk_h;
      end
      
    // This state must be included because the device might possibly
    // delay for up to 10 milliseconds before beginning its clock pulses.
    // During that waiting time, we cannot drive the data (q[0]) because it
    // is possibly 1, which would cause the keyboard to abort its receive
    // and the expected clocks would then never be generated.
    m1_tx_first_wait_clk_l :
      begin
        ps2_data_hi_z <= 0;
        if (~ps2_clk_s) m1_next_state <= m1_tx_clk_l;
        else m1_next_state <= m1_tx_first_wait_clk_l;
      end

    m1_tx_wait_clk_h :
      begin
        enable_timer_5usec <= 1;
        ps2_data_hi_z <= q[0];
        if (ps2_clk_s && timer_5usec_done)
          m1_next_state <= m1_tx_rising_edge_marker;
        else
          m1_next_state <= m1_tx_wait_clk_h;
      end

    m1_tx_rising_edge_marker :
      begin
        ps2_data_hi_z <= q[0];
        m1_next_state <= m1_tx_clk_h;
      end

    m1_tx_clk_h :
      begin
        ps2_data_hi_z <= q[0];
        if (tx_shifting_done) m1_next_state <= m1_tx_wait_keyboard_ack;
        else if (~ps2_clk_s) m1_next_state <= m1_tx_clk_l;
        else m1_next_state <= m1_tx_clk_h;
      end

    m1_tx_clk_l :
      begin
        ps2_data_hi_z <= q[0];
        if (ps2_clk_s) m1_next_state <= m1_tx_wait_clk_h;
        else m1_next_state <= m1_tx_clk_l;
      end

    m1_tx_wait_keyboard_ack :
      begin
        if (~ps2_clk_s && ps2_data_s)
          m1_next_state <= m1_tx_error_no_keyboard_ack;
        else if (~ps2_clk_s && ~ps2_data_s)
          m1_next_state <= m1_tx_done_recovery;
        else m1_next_state <= m1_tx_wait_keyboard_ack;
      end

    m1_tx_done_recovery :
      begin
        if (ps2_clk_s && ps2_data_s) m1_next_state <= m1_rx_clk_h;
        else m1_next_state <= m1_tx_done_recovery;
      end

    m1_tx_error_no_keyboard_ack :
      begin
        tx_error_no_keyboard_ack <= 1;
        if (ps2_clk_s && ps2_data_s) m1_next_state <= m1_rx_clk_h;
        else m1_next_state <= m1_tx_error_no_keyboard_ack;
      end

    default : m1_next_state <= m1_rx_clk_h;
  endcase
end

// State register
always @(posedge clk)
begin : m2_state_register
  if (reset) m2_state <= m2_rx_data_ready_ack;
  else m2_state <= m2_next_state;
end

// State transition logic
always @(m2_state or rx_output_strobe or rx_read or esc_active or esc_flag)
//always @(m2_state , rx_output_strobe , rx_read , esc_active , esc_flag)
begin : m2_state_logic
  
  case (m2_state)
    m2_rx_data_ready_ack:
          begin
            rx_data_ready <= 1'b0;                                                  // nothing doing ...
            if (rx_output_strobe) m2_next_state <= m2_rx_data_ready;                // wait for new PS2 data available
            else m2_next_state <= m2_rx_data_ready_ack;                             // then set data ready output
          end
          
    m2_rx_data_ready:
          begin
            rx_data_ready <= 1'b1;                                                  // flag to receiver that new ascii available
            if (rx_read && (esc_active) )
                begin 
                m2_next_state <= m2_rx_data_escaped_ack;    // if receiver just took ESC then send follow up DATA
                end 
            else if (rx_read) 
                m2_next_state <= m2_rx_data_ready_ack;                // wait for receiver to take it
            else 
                m2_next_state <= m2_rx_data_ready;                                 // then repeat
          end
          
    m2_rx_data_escaped_ack:
          begin
            rx_data_ready <= 0;                                                  // nothing doing ...
            // DATA setup in case below
            if(esc_flag) 
                m2_next_state <= m2_rx_data_escaped;
            else 
                m2_next_state <= m2_rx_data_escaped_ack;                           // data ready to read
          end
                   
    m2_rx_data_escaped:
          begin
            rx_data_ready <= 1;                                                  // flag to receiver that new ascii available
            if (rx_read ) 
                m2_next_state <= m2_rx_data_escaped_done;
            else 
                m2_next_state <= m2_rx_data_escaped;
          end

    m2_rx_data_escaped_done:
        begin
            m2_next_state <= m2_rx_data_ready_ack;                              // start over
        end

    default : m2_next_state <= m2_rx_data_ready_ack;
  endcase
end

// This is the bit counter
always @(posedge clk)
begin
  if (   reset
      || rx_shifting_done
      || (m1_state == m1_tx_wait_keyboard_ack)        // After tx is done.
      ) bit_count <= 0;  // normal reset
  else if (timer_60usec_done
           && (m1_state == m1_rx_clk_h)
           && (ps2_clk_s)
      ) bit_count <= 0;  // rx watchdog timer reset
//  else if (((m1_state == m1_rx_clk_h) && ~ps2_clk_s)  // increment for rx
  else if ( (m1_state == m1_rx_falling_edge_marker)   // increment for rx
           ||(m1_state == m1_tx_rising_edge_marker)   // increment for tx
           )
    bit_count <= bit_count + 1;
end
// This signal is high for one clock at the end of the timer count.
assign rx_shifting_done = (bit_count == `TOTAL_BITS);
assign tx_shifting_done = (bit_count == `TOTAL_BITS-1);

// This is the signal which enables loading of the shift register.
// It also indicates "ack" to the device writing to the transmitter.
assign tx_write_ack_o = (  (tx_write && (m1_state == m1_rx_clk_h))
                         ||(tx_write && (m1_state == m1_rx_clk_l))
                         );

// This is the ODD parity bit for the transmitted word.
assign tx_parity_bit = ~^tx_data;

// This is the shift register
always @(posedge clk)
begin
  if (reset) q <= 0;
  else if (tx_write_ack_o) q <= {1'b1,tx_parity_bit,tx_data,1'b0};
//  else if (((m1_state == m1_rx_clk_h) && ~ps2_clk_s)
  else if ( (m1_state == m1_rx_falling_edge_marker)
           ||(m1_state == m1_tx_rising_edge_marker) )
    q <= {ps2_data_s,q[`TOTAL_BITS-1:1]};
end

// This is the 60usec timer counter
always @(posedge clk)
begin
  if (~enable_timer_60usec) timer_60usec_count <= 0;
  else if (~timer_60usec_done) timer_60usec_count <= timer_60usec_count + 1;
end
assign timer_60usec_done = (timer_60usec_count == (TIMER_60USEC_VALUE_PP - 1));

// This is the 5usec timer counter
always @(posedge clk)
begin
  if (~enable_timer_5usec) timer_5usec_count <= 0;
  else if (~timer_5usec_done) timer_5usec_count <= timer_5usec_count + 1;
end
assign timer_5usec_done = (timer_5usec_count == TIMER_5USEC_VALUE_PP - 1);


// Create the signals which indicate special scan codes received.
// These are the "unlatched versions."
assign extended = (q[8:1] == `EXTEND_CODE) && rx_shifting_done;
assign released = (q[8:1] == `RELEASE_CODE) && rx_shifting_done;

// Store the special scan code status bits
// Not the final output, but an intermediate storage place,
// until the entire set of output data can be assembled.
always @(posedge clk)
begin
  if (reset || rx_output_event)
  begin
    hold_extended <= 0;
    hold_released <= 0;
  end
  else
  begin
    if (rx_shifting_done && extended) hold_extended <= 1;
    if (rx_shifting_done && released) hold_released <= 1;
  end

  if( (tx_write) || (m1_state == m1_tx_wait_keyboard_ack) ) rx_acknowledge <= 0;
  else
  if (rx_shifting_done &&  (q[8:1] == `ACKNOWLEDGE_CODE) ) rx_acknowledge <= 1;

end

// These bits contain the status of the two shift keys
always @(posedge clk)
begin
  if (reset) left_shift_key <= 0;
  else if ((q[8:1] == `LEFT_SHIFT) && rx_shifting_done && ~hold_released)
    left_shift_key <= 1;
  else if ((q[8:1] == `LEFT_SHIFT) && rx_shifting_done && hold_released)
    left_shift_key <= 0;
end

always @(posedge clk)
begin
  if (reset) right_shift_key <= 0;
  else if ((q[8:1] == `RIGHT_SHIFT) && rx_shifting_done && ~hold_released)
    right_shift_key <= 1;
  else if ((q[8:1] == `RIGHT_SHIFT) && rx_shifting_done && hold_released)
    right_shift_key <= 0;
end

assign rx_shift_key_on = left_shift_key || right_shift_key;

// djrm ctrl key detection
always @(posedge clk)
begin
  if (reset) left_ctrl_key <= 0;
  else if ((q[8:1] == `LEFT_CTRL) && rx_shifting_done && ~hold_released)
    left_ctrl_key <= 1;
  else if ((q[8:1] == `LEFT_CTRL) && rx_shifting_done && hold_released)
    left_ctrl_key <= 0;
end

always @(posedge clk)
begin
  if (reset) right_ctrl_key <= 0;
  else if ((q[8:1] == `RIGHT_CTRL) && hold_extended && rx_shifting_done && ~hold_released)
    right_ctrl_key <= 1;
  else if ((q[8:1] == `RIGHT_CTRL) && hold_extended && rx_shifting_done && hold_released)
    right_ctrl_key <= 0;
end

assign rx_ctrl_key_on = left_ctrl_key || right_ctrl_key;

// djrm caps lock and num lock
always @(posedge clk)
begin
  if (reset) caps_state <= 0;
  else if ((q[8:1] == `CAPS_CODE) && rx_shifting_done && hold_released && ~caps_state )
    caps_state <= 1;
  else if ((q[8:1] == `CAPS_CODE) && rx_shifting_done && hold_released && caps_state )
     caps_state <= 0;
end

always @(posedge clk)
begin
  if (reset) num_state <= 0;
  else if ((q[8:1] == `NUM_CODE) && rx_shifting_done && hold_released && ~num_state)
    num_state <= 1;
  else if ((q[8:1] == `NUM_CODE) && rx_shifting_done && hold_released && num_state)
    num_state <= 0;
end

always @(posedge clk)
begin
  if (reset) scroll_state <= 0;
  else if ((q[8:1] == `SCROLL_CODE) && rx_shifting_done && hold_released && ~scroll_state)
    scroll_state <= 1;
  else if ((q[8:1] == `SCROLL_CODE) && rx_shifting_done && hold_released && scroll_state)
    scroll_state <= 0;
end

always @(posedge clk)
begin
  if (reset) update_leds <= 0;
  else if ( ((q[8:1] == `NUM_CODE) && rx_shifting_done && hold_released)
        ||  ((q[8:1] == `CAPS_CODE) && rx_shifting_done && hold_released)
        ||  ((q[8:1] == `SCROLL_CODE) && rx_shifting_done && hold_released) )
    update_leds <= 1;
  else
    update_leds <= 0;
end

assign caps_lock = caps_state;
assign num_lock = num_state;
assign scroll_lock = scroll_state;

// Output the special scan code flags, the scan code and the ascii
always @(posedge clk)
begin
  if (reset)
  begin
    rx_extended <= 0;
    rx_released <= 0;
    rx_scan_code <= 0;
    rx_ascii <= 0;
    esc_reset <= 1;
  end
//  else if(m2_state == m2_rx_data_ready_ack)
//    begin
//    esc_reset <= 0;
//    end  
  else if(m2_state == m2_rx_data_escaped_ack)
  begin
     esc_flag <= 1;   
     rx_ascii <= esc_ascii;
  end
  else if(m2_state == m2_rx_data_escaped)
  begin
     esc_flag <= 0;                                                  // data ready
  end
//  else if (m2_state == m2_rx_data_escaped_done)
//  begin
//     esc_reset <= 1;
//  end
  else if (rx_output_strobe && q[8:1])
  begin
    rx_extended <= hold_extended;
    rx_released <= hold_released;
    rx_scan_code <= q[8:1];
    
    if (ascii > 8'h40 && ascii < 8'h7f && rx_ctrl_key_on)
        rx_ascii = ascii & 'b011111;
    else if (ascii > 8'h40 && ascii < 8'h5B && caps_lock)
        rx_ascii = ascii | 'b0100000;
    else if (ascii > 8'h60 && ascii < 8'h7B && caps_lock)
        rx_ascii <= ascii &  'b1011111;
    else
        rx_ascii <= ascii;  
  end
end

// Store the final rx output data only when all extend and release codes
// are received and the next (actual key) scan code is also ready.
// (the presence of rx_extended or rx_released refers to the
// the current latest scan code received, not the previously latched flags.)
assign rx_output_event  = (rx_shifting_done
                          && ~extended 
                          && ~released
                          );

assign rx_output_strobe = (rx_shifting_done
                          && ~extended 
                          && ~released
                          && ( (TRAP_SHIFT_KEYS_PP == 0) 
                               || ( (q[8:1] != `RIGHT_SHIFT)
                                    &&(q[8:1] != `LEFT_SHIFT)
                                    &&(q[8:1] != `RIGHT_CTRL)
                                    &&(q[8:1] != `LEFT_CTRL)
                                    &&(q[8:1] != `CAPS_CODE)
                                    &&(q[8:1] != `NUM_CODE)
                                    &&(q[8:1] != `SCROLL_CODE)
                                    &&(q[8:1] != `ACKNOWLEDGE_CODE)
                                  )
                             )
                          );

// This part translates the scan code into an ASCII value...
// Only the ASCII codes which I considered important have been included.
// if you want more, just add the appropriate case statement lines...
// (You will need to know the keyboard scan codes you wish to assign.)
// The entries are listed in ascending order of ASCII value.

//assign shift_key_plus_code = {3'b0,rx_shift_key_on,q[8:1]};
// shifted  key 0x1xx
// extended key 0x2xx
// numlock key  0x4xx (xor with shift)
// keycode      0x?nn
assign shift_key_plus_code = {1'b0,(num_lock ^ rx_shift_key_on) , hold_extended, rx_shift_key_on, q[8:1]};
always @(shift_key_plus_code, esc_reset)
begin
//  if (esc_reset) esc_active <= 0;
  
  casez (shift_key_plus_code & 12'h3ff) // ignore num-lock
    12'h?66 : ascii <= 8'h08;  // Backspace ("backspace" key)
    12'h?0d : ascii <= 8'h09;  // Horizontal Tab
    12'h?5a : ascii <= 8'h0d;  // Carriage return ("enter" key)
    12'h?76 : ascii <= 8'h1b;  // Escape ("esc" key)
    12'h?29 : ascii <= 8'h20;  // Space
    12'h116 : ascii <= 8'h21;  // !
    12'h126 : ascii <= 8'h23;  // #
    12'h125 : ascii <= 8'h24;  // $
    12'h12e : ascii <= 8'h25;  // %
    12'h13d : ascii <= 8'h26;  // &
    12'h052 : ascii <= 8'h27;  // '
    12'h146 : ascii <= 8'h28;  // (
    12'h145 : ascii <= 8'h29;  // )
    12'h13e : ascii <= 8'h2a;  // *
    12'h155 : ascii <= 8'h2b;  // +
    12'h041 : ascii <= 8'h2c;  // ,
    12'h04e : ascii <= 8'h2d;  // -
    12'h049 : ascii <= 8'h2e;  // .
    12'h04a : ascii <= 8'h2f;  // /
    12'h045 : ascii <= 8'h30;  // 0
    12'h016 : ascii <= 8'h31;  // 1
    12'h01e : ascii <= 8'h32;  // 2
    12'h026 : ascii <= 8'h33;  // 3
    12'h025 : ascii <= 8'h34;  // 4
    12'h02e : ascii <= 8'h35;  // 5
    12'h036 : ascii <= 8'h36;  // 6
    12'h03d : ascii <= 8'h37;  // 7
    12'h03e : ascii <= 8'h38;  // 8
    12'h046 : ascii <= 8'h39;  // 9
    12'h14c : ascii <= 8'h3a;  // :
    12'h04c : ascii <= 8'h3b;  // ;
    12'h141 : ascii <= 8'h3c;  // <
    12'h055 : ascii <= 8'h3d;  // =
    12'h149 : ascii <= 8'h3e;  // >
    12'h14a : ascii <= 8'h3f;  // ?
    12'h11c : ascii <= 8'h41;  // A
    12'h132 : ascii <= 8'h42;  // B
    12'h121 : ascii <= 8'h43;  // C
    12'h123 : ascii <= 8'h44;  // D
    12'h124 : ascii <= 8'h45;  // E
    12'h12b : ascii <= 8'h46;  // F
    12'h134 : ascii <= 8'h47;  // G
    12'h133 : ascii <= 8'h48;  // H
    12'h143 : ascii <= 8'h49;  // I
    12'h13b : ascii <= 8'h4a;  // J
    12'h142 : ascii <= 8'h4b;  // K
    12'h14b : ascii <= 8'h4c;  // L
    12'h13a : ascii <= 8'h4d;  // M
    12'h131 : ascii <= 8'h4e;  // N
    12'h144 : ascii <= 8'h4f;  // O
    12'h14d : ascii <= 8'h50;  // P
    12'h115 : ascii <= 8'h51;  // Q
    12'h12d : ascii <= 8'h52;  // R
    12'h11b : ascii <= 8'h53;  // S
    12'h12c : ascii <= 8'h54;  // T
    12'h13c : ascii <= 8'h55;  // U
    12'h12a : ascii <= 8'h56;  // V
    12'h11d : ascii <= 8'h57;  // W
    12'h122 : ascii <= 8'h58;  // X
    12'h135 : ascii <= 8'h59;  // Y
    12'h11a : ascii <= 8'h5a;  // Z
    12'h054 : ascii <= 8'h5b;  // [
    12'h05b : ascii <= 8'h5d;  // ]
    12'h136 : ascii <= 8'h5e;  // ^
    12'h14e : ascii <= 8'h5f;  // _    
    12'h01c : ascii <= 8'h61;  // a
    12'h032 : ascii <= 8'h62;  // b
    12'h021 : ascii <= 8'h63;  // c
    12'h023 : ascii <= 8'h64;  // d
    12'h024 : ascii <= 8'h65;  // e
    12'h02b : ascii <= 8'h66;  // f
    12'h034 : ascii <= 8'h67;  // g
    12'h033 : ascii <= 8'h68;  // h
    12'h043 : ascii <= 8'h69;  // i
    12'h03b : ascii <= 8'h6a;  // j
    12'h042 : ascii <= 8'h6b;  // k
    12'h04b : ascii <= 8'h6c;  // l
    12'h03a : ascii <= 8'h6d;  // m
    12'h031 : ascii <= 8'h6e;  // n
    12'h044 : ascii <= 8'h6f;  // o
    12'h04d : ascii <= 8'h70;  // p
    12'h015 : ascii <= 8'h71;  // q
    12'h02d : ascii <= 8'h72;  // r
    12'h01b : ascii <= 8'h73;  // s
    12'h02c : ascii <= 8'h74;  // t
    12'h03c : ascii <= 8'h75;  // u
    12'h02a : ascii <= 8'h76;  // v
    12'h01d : ascii <= 8'h77;  // w
    12'h022 : ascii <= 8'h78;  // x
    12'h035 : ascii <= 8'h79;  // y
    12'h01a : ascii <= 8'h7a;  // z
    12'h154 : ascii <= 8'h7b;  // {
`ifndef ALT_LAYOUT
    // uk keyboard layout    
    12'h00e : ascii <= 8'h60;  // `
    12'h10e : ascii <= 8'h7c;  // |
    12'h061 : ascii <= 8'h5c;  // \
    12'h161 : ascii <= 8'h7c;  // |
    12'h152 : ascii <= 8'h40;  // @
    12'h11e : ascii <= 8'h22;  // "
    12'h05d : ascii <= 8'h23;  // #
    12'h15d : ascii <= 8'h7e;  // ~
`else    
    // other keyboard layout (US?)
    12'h152 : ascii <= 8'h22;  // "
    12'h11e : ascii <= 8'h40;  // @
    12'h05d : ascii <= 8'h5c;  // \
    12'h15d : ascii <= 8'h7c;  // |
    12'h10e : ascii <= 8'h7e;  // ~
`endif    
    
    12'h15b : ascii <= 8'h7d;  // }
    //default : ascii <= 8'h2e;  // '.' used for unlisted characters.
    default : ascii <= 8'h3f;  // ?
  endcase
  
  // numeric keypad decode - uses num-lock status
// shifted  key 0x1xx
// extended key 0x2xx
// numlock key  0x4xx (xor with shift)
// keycode      0x?nn
  
  case (shift_key_plus_code & 12'h6ff) // ignore usual shift
    // numeric keypad digits (shift ^ numlock)
    12'h471 : ascii <= 8'h2e;  // .      (DEL on numeric keypad)
    12'h470 : ascii <= 8'h30;  // 0
    12'h469 : ascii <= 8'h31;  // 1
    12'h472 : ascii <= 8'h32;  // 2
    12'h47a : ascii <= 8'h33;  // 3
    12'h46b : ascii <= 8'h34;  // 4
    12'h473 : ascii <= 8'h35;  // 5
    12'h474 : ascii <= 8'h36;  // 6
    12'h46c : ascii <= 8'h37;  // 7
    12'h475 : ascii <= 8'h38;  // 8
    12'h47d : ascii <= 8'h39;  // 9
        
    12'h071 : ascii <= 8'h7f;  // delete (Del on numeric keypad)
    12'h070 : ascii <= 8'h49;  // delete (Ins on numeric keypad)
    12'h072 : ascii <= 8'h44;  // 2 D
    12'h06b : ascii <= 8'h4c;  // 4 L
    12'h074 : ascii <= 8'h52;  // 6 R
    12'h075 : ascii <= 8'h55;  // 8 U
    12'h06c : ascii <= 8'h48;  // 7 H (home)
    12'h069 : ascii <= 8'h45;  // 1 E (end)
    12'h07d : ascii <= 8'h50;  // 9 P (p Up)
    12'h07a : ascii <= 8'h4e;  // 3 N (p Dn)
    12'h073 :               ;  // 5 (nothing)
  endcase

  case (shift_key_plus_code & 12'h2ff) // ignore num-lock and shift, accept extended keys
    // cursor keys
    12'h272 : begin esc_active <= 1; ascii <= 8'h1b; esc_ascii <= 8'h42; end // down
    12'h26b : begin esc_active <= 1; ascii <= 8'h1b; esc_ascii <= 8'h44; end // left
    12'h274 : begin esc_active <= 1; ascii <= 8'h1b; esc_ascii <= 8'h43; end // right
    12'h275 : begin esc_active <= 1; ascii <= 8'h1b; esc_ascii <= 8'h41; end // up
    
      // numeric keypad operators
    12'h24a : ascii <= 8'h2f;  // /
    12'h07c : ascii <= 8'h2a;  // *
    12'h07b : ascii <= 8'h2d;  // -
    12'h079 : ascii <= 8'h2b;  // +
    12'h25a : begin esc_active <= 0; ascii <= 8'h0d; esc_ascii <= 8'h0; end // ("enter" key)

    // block keys (ins,del, home, end, pgup, pgdn)
    12'h270 : ascii <= 8'h69;  // i (inserte)
    12'h271 : ascii <= 8'h7f;  //   (delete)
    12'h26c : ascii <= 8'h68;  // h (home)
    12'h269 : ascii <= 8'h65;  // e (end)
    12'h27d : ascii <= 8'h70;  // p (p Up)
    12'h27a : ascii <= 8'h6e;  // n (p Dn)

  endcase
  
end


endmodule

