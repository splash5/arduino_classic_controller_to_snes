#include <avr/wdt.h>

#define ANALOG_DEAD_ZONE  0x10
 // 200KHz is much more stable on some 3rd-party controller
#define IIC_SPEED         300000

#define SNES_LATCH_PIN    2
#define SNES_CLK_PIN      7
#define SNES_DATA_PIN     8
#define EXTENSION_CONNECT 3

#include <Wire.h>

volatile uint8_t classic_controller_state = 0;
volatile uint16_t snes_button_state = 0xffff;
volatile uint16_t rapid_fire_buttons = 0x0000;
volatile uint16_t rapid_fire_speed = 0x0000;  // 0: slow, 1: fast, each bit maps buttons
volatile uint8_t rapid_fire_count = 0;

byte i2c_data[32];

void setup()
{
  // for debugging
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(EXTENSION_CONNECT, INPUT);
  pinMode(SNES_LATCH_PIN, INPUT);
  pinMode(SNES_CLK_PIN, INPUT);
  pinMode(SNES_DATA_PIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(SNES_LATCH_PIN, LOW);  // disable internal pullup
  digitalWrite(SNES_CLK_PIN, LOW);    // disable internal pullup
  digitalWrite(SNES_DATA_PIN, HIGH);
  
  // initialze i2c lib
  // begin i2c
  Wire.begin();
  Wire.setClock(IIC_SPEED);
      
  // initialize classic controller if connected when power-on
  controller_connection_state_changed();

  // detect connection using interrupt
  attachInterrupt(digitalPinToInterrupt(SNES_LATCH_PIN), snes_latch_triggered, RISING);
  attachInterrupt(digitalPinToInterrupt(EXTENSION_CONNECT), controller_connection_state_changed, CHANGE);

  wdt_enable(WDTO_30MS);
}

void loop()
{
  wdt_reset();

  switch (classic_controller_state)
  {
    case 1:
    {
      if (initialize_classic_controller())
      {
        classic_controller_state = 2;
      }

      break;
    }
    case 2:
    {
      if (fetch_classic_controller_button_state())
      {
        arrange_to_snes_button_state();
      }
      else
      {
        // TODO force reset?
        controller_connection_state_changed();
//        snes_button_state = 0xffff;
      }
      
      break;
    }
  }
}

bool initialize_classic_controller()
{
  byte i = 0;
  
  Wire.beginTransmission(0x52);
  Wire.write(0xf0);
  Wire.write(0x55);
  if (Wire.endTransmission(true))
    goto initialize_error_exit;

  delayMicroseconds(100);
  Wire.beginTransmission(0x52);
  Wire.write(0xfb);
  Wire.write(0x00);
  if (Wire.endTransmission(true))
    goto initialize_error_exit;

  delayMicroseconds(100);
  Wire.beginTransmission(0x52);
  Wire.write(0xfe);
  Wire.write(0x03);
  if (Wire.endTransmission(true))
    goto initialize_error_exit;

  delayMicroseconds(100);
  Wire.beginTransmission(0x52);
  Wire.write(0xfa);
  if (Wire.endTransmission(true))
    goto initialize_error_exit;

  delayMicroseconds(300);
  if (Wire.requestFrom(0x52, 6) != 6)
    goto initialize_error_exit;
  
  while (Wire.available() > 0)
    i2c_data[i++] = Wire.read();

  while (i < 6)
    i2c_data[i++] = 0x00;
/* for debug
  Serial.print(i2c_data[0], HEX); Serial.print(' ');
  Serial.print(i2c_data[1], HEX); Serial.print(' ');
  Serial.print(i2c_data[2], HEX); Serial.print(' ');
  Serial.print(i2c_data[3], HEX); Serial.print(' ');
  Serial.print(i2c_data[4], HEX); Serial.print(' ');
  Serial.println(i2c_data[5], HEX);
*/
  if ((i2c_data[2] == 0xa4 && i2c_data[3] == 0x20) &&
      (i2c_data[1] == 0x00 && i2c_data[5] == 0x01) &&
      // classic controller: 0x00, classic controller pro: 0x01
      (i2c_data[0] == 0x00 || i2c_data[0] == 0x01) &&
      // I always got 0x03 here but wiibrew says this should be 0x01...
      (i2c_data[4] & 0x01))
  {
      // LED on
      PORTB = PORTB | 0x20;
      return true;
  }

initialize_error_exit:
  // LED off
  PORTB = PORTB & 0xdf;
  return false;
}

bool fetch_classic_controller_button_state()
{
    register byte i = 0;

    delayMicroseconds(100);
    Wire.beginTransmission(0x52);
    Wire.write(0x00);    
    if (Wire.endTransmission(true))
      goto fetch_error_exit;

    delayMicroseconds(300);
    if (Wire.requestFrom(0x52, 21) != 21)
      goto fetch_error_exit;

    while (Wire.available() > 0)
      i2c_data[i++] = Wire.read();

    while (i < 21)
      i2c_data[i++] = 0xff;

    return true;
fetch_error_exit:
  return false;
}

void arrange_to_snes_button_state() 
{
  /*
  byte lx = i2c_data[0];
  byte rx = i2c_data[1];
  byte ly = i2c_data[2];
  byte ry = i2c_data[3];
  byte lt = i2c_data[4];
  byte rt = i2c_data[5];
  */
  asm volatile
  (// some button remap things here
   "sbrs %[i2c_7], 7\n\t"     // ZL to L
   "cbr %[i2c_6], 0x20\n\t"
   "sbrs %[i2c_7], 2\n\t"     // ZR to R
   "cbr %[i2c_6], 0x02\n\t"
   // check analog LX
   "cpi %[i2c_0], %[MINUS_THRESHOLD]\n\t"   // check LX < MINUS_THRESHOLD
   "brsh check_lx_plus_%=\n\t"
   "cbr %[i2c_7], 0x02\n\t"   // LX- to LEFT
   "rjmp end_checking_lx_%=\n\t"
   "check_lx_plus_%=:\n\t"
   "cpi %[i2c_0], %[PLUS_THRESHOLD]\n\t"    // check LX >= PLUS_THRESHOLD
   "brlo end_checking_lx_%=\n\t"
   "cbr %[i2c_6], 0x80\n\t"   // LX+ to RIGHT
   "end_checking_lx_%=:\n\t"
   // check analog LY
   "cpi %[i2c_2], %[MINUS_THRESHOLD]\n\t"   // check LY < MINUS_THRESHOLD
   "brsh check_ly_plus_%=\n\t"
   "cbr %[i2c_6], 0x40\n\t"   // LY- to DOWN
   "rjmp end_checking_ly_%=\n\t"
   "check_ly_plus_%=:\n\t"
   "cpi %[i2c_2], %[PLUS_THRESHOLD]\n\t"    // check LY >= PLUS_THRESHOLD
   "brlo end_checking_ly_%=\n\t"
   "cbr %[i2c_7], 0x01\n\t"   // LY+ to UP
   "end_checking_ly_%=:\n\t"
   // finish remapping buttons
   // start filling snes controller data
   // put controller id here
   "ldi %B[snes_button_state], 0xff\n\t"
   // start filling button state
   "bst %[i2c_7], 6\n\t"       // check button B
   "bld %A[snes_button_state], 0\n\t"
   "bst %[i2c_7], 5\n\t"       // check button Y
   "bld %A[snes_button_state], 1\n\t"
   "bst %[i2c_6], 4\n\t"       // check button SELECT
   "bld %A[snes_button_state], 2\n\t"
   "bst %[i2c_6], 2\n\t"       // check button START
   "bld %A[snes_button_state], 3\n\t"
   "bst %[i2c_7], 0\n\t"       // check button UP
   "bld %A[snes_button_state], 4\n\t"
   "bst %[i2c_6], 6\n\t"       // check button DOWN
   "bld %A[snes_button_state], 5\n\t"
   "bst %[i2c_7], 1\n\t"       // check button LEFT
   "bld %A[snes_button_state], 6\n\t"
   "bst %[i2c_6], 7\n\t"       // check button RIGHT
   "bld %A[snes_button_state], 7\n\t"
   "bst %[i2c_7], 4\n\t"       // check button A
   "bld %B[snes_button_state], 0\n\t"
   "bst %[i2c_7], 3\n\t"       // check button X
   "bld %B[snes_button_state], 1\n\t"
   "bst %[i2c_6], 5\n\t"       // check button L
   "bld %B[snes_button_state], 2\n\t"
   "bst %[i2c_6], 1\n\t"       // check button R
   "bld %B[snes_button_state], 3\n\t"
   : [snes_button_state] "=&a" (snes_button_state)
   : [i2c_0] "a" (i2c_data[0]), [i2c_2] "a" (i2c_data[2]), [i2c_6]"a" (i2c_data[6]), [i2c_7] "a" (i2c_data[7]), [MINUS_THRESHOLD] "M" (0x80 - ANALOG_DEAD_ZONE), [PLUS_THRESHOLD] "M" (0x80 + ANALOG_DEAD_ZONE)
  );
}

// trigger when changed
void controller_connection_state_changed()
{
  asm volatile
  (// init classic_controller_state value
   "ldi %[controller_state], 0x00\n\t"
   // clear snes_button_state
   "ldi %A[snes_button_state], 0xff\n\t"
   "ldi %B[snes_button_state], 0xff\n\t"
   // get PIND3 state
   "bst %[pind], 3\n\t"
   // set to classic_controller_state
   "bld %[controller_state], 0\n\t"
   "cbi %[portb], 0\n\t"  // set snes data pin as (hi-z or LOW)
   "brtc controller_disconnected_%=\n\t"
   "sbi %[ddrb], 0\n\t"   // set snes data pin as output
   "rjmp end_of_controller_connection_state_changed_%=\n\t"
   "controller_disconnected_%=:\n\t"
   "cbi %[portb], 5\n\t"  // turn LED off
   "cbi %[ddrb], 0\n\t"   // set snes data pin as input
   "end_of_controller_connection_state_changed_%=:"
   : [controller_state] "=&a" (classic_controller_state), [snes_button_state] "=&a" (snes_button_state)
   : [pind] "a" (PIND), [portb] "I" (_SFR_IO_ADDR(PORTB)), [ddrb] "I" (_SFR_IO_ADDR(DDRB))
  );
}

void snes_latch_triggered()
{
  register byte clk_count, byte_count, abort_count;
  register byte button_state_8, rapid_fire_enable_8, rapid_fire_speed_8;

  // check if controller is disconnected
  asm volatile
  (
   "and %[classic_controller_state], %[classic_controller_state]\n\t"
   "breq end_of_snes_latch_triggered\n\t"
   :: [classic_controller_state] "r" (classic_controller_state)
  );

  asm volatile
  (
   "ldi %[byte_count], 0x00\n\t"
   "send_snes_data_one_byte_%=:\n\t"
   // load snes_button_state
   "ld %[button_state_8], %a[button_ptr]+\n\t"
   // load rapid_fire_buttons
   "ld %[rapid_fire_enable_8], %a[turbo_ptr]+\n\t"
   // load rapid_fire_speed
   "ld %[rapid_fire_speed_8], %a[turbo_speed_ptr]+\n\t"
   // reset clk_count
   "ldi %[clk_count], 0x00\n\t"
   "send_snes_data_one_bit_%=:\n\t"
   // start sending one bit
   "bst %[rapid_fire_enable_8], 0\n\t"
   "lsr %[rapid_fire_enable_8]\n\t"
   "lsr %[button_state_8]\n\t"
   // if button is not pressed -> 1
   "brcs force_send_snes_data_1_%=\n\t"
   // check if rapid fire is enabled for this button
   "brtc force_send_snes_data_0_%=\n\t"
   // check if counter has reached
   // rapid fire speed control here
   // copy rapid_fire_count
   "mov __tmp_reg__, %[rapid_fire_count]\n\t"
   "lsr __tmp_reg__\n\t"
   // if fast speed, no more shift
   "sbrs %[rapid_fire_speed_8], 0\n\t"
   "lsr __tmp_reg__\n\t"
   "bst __tmp_reg__, 0\n\t"
   "brts force_send_snes_data_1_%=\n\t"
   // restore carry flag (which is zero in button pressed state)
   "clc\n\t"
   "force_send_snes_data_0_%=:\n\t"
   "cbi %[portb], 0\n\t"
   "brcc send_snes_data_one_bit_finish_%=\n\t"
   "force_send_snes_data_1_%=:\n\t"
   "sbi %[portb], 0\n\t"
   "send_snes_data_one_bit_finish_%=:\n\t"
   // next button speed
   "lsr %[rapid_fire_speed_8]\n\t"
   // clear abort_count
   "ldi %[abort_count], 0x00\n\t"
   // end send one bit
   // start polling clk signal
   // wait for clk falling
   "wait_for_clk_falling_%=:\n\t"
   "cpi %[abort_count], 192\n\t"      // check if reach to abort count (40 is minimun, 192 for super momotaru dentetsu 2 -- super long latch)
   "brsh end_of_snes_latching_%=\n\t" // force leave latching
   "inc %[abort_count]\n\t"
   // check clk falling
   "sbic %[pind], 7\n\t"
   "rjmp wait_for_clk_falling_%=\n\t"
   // wait for clk rising
   "wait_for_clk_rising_%=:\n\t"
   "sbis %[pind], 7\n\t"
   "rjmp wait_for_clk_rising_%=\n\t"
   // check if 8 bits sent
   "inc %[clk_count]\n\t"
   "cpi %[clk_count], 8\n\t"
   "brlo send_snes_data_one_bit_%=\n\t"
   // check if 2 bytes sent
   "inc %[byte_count]\n\t"
   "cpi %[byte_count], 2\n\t"
   "brlo send_snes_data_one_byte_%=\n\t"
   "end_of_snes_latching_%=:\n\t"
   // data set to low
   "cbi %[portb], 0\n\t"
   "inc %[rapid_fire_count]\n\t"
   : [rapid_fire_count] "+&a" (rapid_fire_count), [button_state_8] "=a" (button_state_8), [rapid_fire_enable_8] "=a" (rapid_fire_enable_8), [rapid_fire_speed_8] "=a" (rapid_fire_speed_8), [abort_count] "=&r" (abort_count), [clk_count] "=&r" (clk_count), [byte_count] "=&r" (byte_count)
   : [portb] "I" (_SFR_IO_ADDR(PORTB)), [pind] "I" (_SFR_IO_ADDR(PIND)), [button_ptr] "e" (&snes_button_state), [turbo_ptr] "e" (&rapid_fire_buttons), [turbo_speed_ptr] "e" (&rapid_fire_speed)
  );

  asm volatile("end_of_snes_latch_triggered:\n\t");
}
