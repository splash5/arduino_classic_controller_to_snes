#include <Wire.h>

volatile byte classic_controller_state = 0;
volatile byte rapid_fire_count = 0;
volatile word snes_button_state = 0xffff;

byte i2c_data[32];

void setup()
{
  Serial.begin(115200);
  
  // for debugging
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(3, INPUT);    // wii extension connect
  pinMode(2, INPUT);    // snes latch input
  pinMode(7, INPUT);    // snes clk input
  pinMode(8, OUTPUT);   // snes data output

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(2, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);
  
  // initialze i2c lib
  // begin i2c
  Wire.begin();
  Wire.setClock(200000); // 200KHz is much more stable on some 3rd-party controller
      
  // initialize classic controller if connected when power-on
  controller_connection_state_changed();

  // detect connection using interrupt
  attachInterrupt(digitalPinToInterrupt(3), controller_connection_state_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), snes_latch_triggered, RISING);
}

void loop()
{
  switch (classic_controller_state)
  {
    case 1:
    {
      if (initialize_classic_controller())
        classic_controller_state = 2;

      break;
    }
    case 2:
    {
      if (!fetch_classic_controller_button_state())
      {
        snes_button_state = 0xffff;

        // blink led
        PORTB = PORTB | 0x20;
        delay(1000);
        PORTB = PORTB & 0xdf;
      }
      else
      {
        arrange_to_snes_button_state();
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
  return false;
}

bool fetch_classic_controller_button_state()
{
    byte i = 0;
  
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
   // maps ZL/ZR to L/R
   "sbrs %2, 7\n\t"
   "cbr %1, 0x20\n\t"
   "sbrs %2, 2\n\t"
   "cbr %1, 0x02\n\t"
   // put controller id here
   "sbr %B0, 0xf0\n\t"
   // start filling button state
   "bst %2, 6\n\t"       // check button B
   "bld %A0, 0\n\t"
   "bst %2, 5\n\t"       // check button Y
   "bld %A0, 1\n\t"
   "bst %1, 4\n\t"       // check button SELECT
   "bld %A0, 2\n\t"
   "bst %1, 2\n\t"       // check button START
   "bld %A0, 3\n\t"
   "bst %2, 0\n\t"       // check button UP
   "bld %A0, 4\n\t"
   "bst %1, 6\n\t"       // check button DOWN
   "bld %A0, 5\n\t"
   "bst %2, 1\n\t"       // check button LEFT
   "bld %A0, 6\n\t"
   "bst %1, 7\n\t"       // check button RIGHT
   "bld %A0, 7\n\t"
   "bst %2, 4\n\t"       // check button A
   "bld %B0, 0\n\t"
   "bst %2, 3\n\t"       // check button X
   "bld %B0, 1\n\t"
   "bst %1, 5\n\t"       // check button L
   "bld %B0, 2\n\t"
   "bst %1, 1\n\t"       // check button R
   "bld %B0, 3\n\t"
   : "=&a" (snes_button_state) : "a" (i2c_data[6]), "a" (i2c_data[7])
  );
}

// trigger when changed
void controller_connection_state_changed()
{
  asm volatile
  (// init classic_controller_state value
   "cbr %0, 0xff\n\t"
   // get PIND3 state
   "bst %2, 3\n\t"
   // set to classic_controller_state
   "bld %0, 0\n\t"
   "brts controller_connection_state_changed_%=\n\t"
   // clear snes_button_state
   "sbr %A1, 0xff\n\t"
   "sbr %B1, 0xff\n\t"
   // turn LED off
   "cbi %3, 5\n\t"
   "controller_connection_state_changed_%=:\n\t"
   : "=&a" (classic_controller_state), "=&a" (snes_button_state)
   : "a" (PIND), "I" (_SFR_IO_ADDR(PORTB))
  );
}

void snes_latch_triggered()
{
  register byte clk_count, byte_count, abort_count;
  register byte button_state_8;

  asm volatile
  ("ldi %[byte_count], 0x00\n\t"
   "send_snes_data_one_byte_%=:\n\t"
   // load snes_button_state
   "ld %[button_state_8], %a[ptr]+\n\t"
   // reset clk_count
   "ldi %[clk_count], 0x00\n\t"
   "send_snes_data_one_bit_%=:\n\t"
   // start sending one bit
   "lsr %[button_state_8]\n\t"
   "brcs send_snes_data_1_%=\n\t"
   "cbi %[portb], 0\n\t"
   "send_snes_data_1_%=:\n\t"
   "brcc send_snes_data_one_bit_finish_%=\n\t"
   "sbi %[portb], 0\n\t"
   "send_snes_data_one_bit_finish_%=:\n\t"
   // clear abort_count
   "ldi %[abort_count], 0x00\n\t"
   // end send one bit
   // start polling clk signal
   // wait for clk falling
   "wait_for_clk_falling_%=:\n\t"
   "cpi %[abort_count], 64\n\t"       // check if reach to abort count (40 is minimun)
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
   : [rapid_fire_count] "+&a" (rapid_fire_count), [button_state_8] "=a" (button_state_8), [abort_count] "=&r" (abort_count), [clk_count] "=&r" (clk_count), [byte_count] "=&r" (byte_count)
   : [portb] "I" (_SFR_IO_ADDR(PORTB)), [pind] "I" (_SFR_IO_ADDR(PIND)), [ptr] "e" (&snes_button_state)
  );
}
