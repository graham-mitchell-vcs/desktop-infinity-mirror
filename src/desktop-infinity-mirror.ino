/*
 * v1.1
 *
 * This program drives the Core Electronics Desktop Infinity Mirror Kit
 * Powered by Core Electronics
 * 2017
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "Particle.h"
#include <math.h>
#include <neopixel.h>

SYSTEM_MODE(SEMI_AUTOMATIC);
// SYSTEM_MODE(AUTOMATIC);


/*******************************************************************************
 * Hardware Definitions
 *
 ******************************************************************************/

// LED strip
const int strip_pin = 0; // The digital pin that drives the LED strip
const int num_leds = 44; // Number of LEDs in the strip. Shouldn't need changing unless you hack the hardware

const int pot_1 = 0; // Potentiometer pin selects the mode

const int ADC_precision = 4095; // Particle use 12bit ADCs. If you wish to port to a different platform you might need to redefine the ADC precision eg: 1023 for Arduino UNO



/*******************************************************************************
 * Global Variables
 *
 ******************************************************************************/

// States for the state-machine
enum statevar {
    state_off,
    state_rainbow,
    state_brightness,
    state_comet,
    state_solid,
    state_scroll,
};

static uint8_t state; // The state that the user demands
static uint8_t state_current; // The state currently being executed
// Adafruit_NeoPixel strip(num_leds, strip_pin);
Adafruit_NeoPixel strip(num_leds, strip_pin, WS2812B);
static uint32_t ledBuffer[num_leds]; // Buffer for storing (and then scaling if necessary) LED R,G,B values.

static float userBright = 1.0; // User-set brightness [0 - 1] // TODO roll into LED-strip object

/*******************************************************************************
 * SETUP
 *
 ******************************************************************************/

void setup()
{
    strip.begin();
    strip.show(); // Initialize all pixels to off
}



/*******************************************************************************
 * LOOP
 *
 ******************************************************************************/

void loop()
{
  static uint32_t userColour = Wheel(0); // User-set colour TODO roll into LED-strip object



    if (System.buttonPushed() > 1) {
        if( !Particle.connected() ){
            Particle.connect();
        }
    }


    // Read potentiometer values for user-input
    state = getState(pot_1);    // Select the operation mode
    //int opt1 = analogRead(pot_2);   // Select animation speed
    //int opt2 = analogRead(pot_3);   // A general-purpose option for other effects
    int opt1 = 1023;
    int opt2 = 1023;


    // State Machine
    switch(state){
        case state_off:
            clearStrip();   // "Off" state.
            break;

        case state_rainbow:
            rainbow(); // Adafruit's rainbow demo, modified for seamless wraparound. We are passing the Pot # instead of the option because delay value needs to be updated WITHIN the rainbow function. Not just at the start of each main loop.
            break;

        case state_brightness:
          brightness(userColour);
          break;

        case state_comet:
            demo(userColour); // An under-construction comet demo.
            break;

        case state_solid:
            solid(userColour);
            break;

        case state_scroll:
            userColour = scroll();
            break;

        default:
            break;

    }
}


/*******************************************************************************
 * Functions
 *
 ******************************************************************************/

// Break potentiometer rotation into four sectors for setting mode
uint8_t getState(int pot){
  // TODO: find better, more flexible method of defining pot sectors?
    float val = float(analogRead(pot)) / float(ADC_precision);

    // TODO remove magic numbers
    if (val < 0.05) {
        return state_off;
    } else if (val < 0.25) {
        return state_rainbow;
    }else if (val < 0.5) {
      return state_brightness;
    } else if (val < 0.75) {
        return state_solid;
    } else if (val < 0.95) {
      return state_comet;
    } else {
      return state_scroll;
    }
}


/* Run the comet demo
 * This feature is largely experimental and quite incomplete.
 * The idea is to involve multiple comets that can interact by colour-addition
 */
void demo(uint32_t colour){
    state_current = state_comet;
    uint16_t i, j, k;
    uint16_t ofs = 15;

    for (j=0; j<strip.numPixels(); j++){
        clearStrip();

        comet(j,1, colour);

        strip.show();
        delay(30);
        if(getState(pot_1) != state_current) break; // Check if mode knob is still on this mode
    }
}


/*
 * Draw a comet on the strip and handle wrapping gracefully.
 * Arguments:
 *      - pos: the pixel index of the comet's head
 *      - dir: the direction that the tail should point
 *
 * TODO:
 *      - Handle direction gracefully. In the works but broken.
 *      - Handle multiple comets
 */
void comet(uint16_t pos, bool dir, uint32_t colour) {
    float headBrightness = 255;                 // Brightness of the first LED in the comet
    uint8_t bright = uint8_t(headBrightness);   // Initialise the brightness variable
    uint16_t len = 20;                          // Length of comet tail
    double lambda = 0.3;                        // Parameter that effects how quickly the comet tail dims
    double dim = lambda;                        // initialise exponential decay function

    //strip.setPixelColor(pos, strip.Color(0,bright,0)); // Head of the comet

    // Extract colour channels
    uint8_t headB = colour & 0xFF;
    uint8_t headG = (colour >> 8) & 0xFF;
    uint8_t headR = (colour >> 16) & 0xFF;
    uint8_t R = headR;
    uint8_t G = headG;
    uint8_t B = headB;

    strip.setPixelColor(pos, colour); // Head of the comet

    if(dir) {
        for(uint16_t i=1; i<len; i++){
            // Figure out if the current pixel is wrapped across the strip ends or not, light that pixel
            if( pos - i < 0 ){ // Wrapped
                strip.setPixelColor(strip.numPixels()+pos-i, strip.Color(R,G,B));
            } else { // Not wrapped
                strip.setPixelColor(pos-i, strip.Color(R,G,B));
            }
            R = uint8_t(headR * exp(-dim)); // Exponential decay function to dim tail LEDs
            G = uint8_t(headG * exp(-dim));
            B = uint8_t(headB * exp(-dim));
            dim += lambda;
        }

    } else { // Comet is going backwards *** BROKEN: TODO fix ***
        // for(uint16_t i=1; i<len; i++){
        //     // Figure out if the current pixel is wrapped across the strip ends or not, light that pixel
        //     if( pos + i > strip.numPixels() ){ // Wrapped
        //         strip.setPixelColor(strip.numPixels()-pos-i, strip.Color(0,bright,0));
        //     } else { // Not wrapped
        //         strip.setPixelColor(pos+i, strip.Color(0,bright,0));
        //     }
        //     // Dim the tail of the worm. This probably isn't the best way to do it, but it'll do for now.
        //     // TODO: dim while respecting the length of the worm. For long worms this will dim to zero before the end of worm is reached.
        //     bright *= 0.75;
        // }
    }
}


void clearStrip(void){
    uint16_t i;
    for(i=0; i<strip.numPixels(); i++){
            strip.setPixelColor(i, strip.Color(0,0,0));
        }
        strip.show();
        delay(1);
}


void rainbow() {
//   uint16_t j;
  float i, baseCol;
  float colStep = 256.0 / strip.numPixels();

  for(baseCol=0; baseCol<256; baseCol++) { // Loop through all colours
    for(i=0; i<strip.numPixels(); i++) {   // Loop through all pixels
        // strip.setPixelColor( i, Wheel(int(i*(colStep)+baseCol) & 255) ); // This line seamlessly wraps the colour around the table.
        setPixel( i, Wheel(int(i*(colStep)+baseCol) & 255) ); // This line seamlessly wraps the colour around the table.
    }
    // strip.show();
    update();
    delay(42);

    if(getState(pot_1) != state_rainbow) break; // Check if mode knob is still on this mode
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


// Display a single solid colour from the Wheel(), or show white with variable brightness
void solid(uint32_t colour){
    // Do not set state_current here.
    uint16_t i;
    // User-defined colour
    for(i=0; i<strip.numPixels(); i++){
      setPixel(i, colour);
    }

    update();
    delay(25);
}

// Scroll through the colour wheel for all LEDs. Also allows user to set a desired colour for other modes.
uint32_t scroll() {
  static int colPos = 0; // Position in colour wheel
  uint32_t colour = Wheel(colPos++ & 255);

  // Show the colour
  solid(colour);

  return colour; // Return the set colour for use in other sequences.

}

// Fade the brightness up <-> down and update a brightness parameter for other modes.
void brightness(uint32_t col) {
  state_current = state_brightness;
  // glow on to max
  for ( userBright; userBright < 1.0; userBright += 0.05 ){
    solid(col);
    if(getState(pot_1) != state_current) break; // Check if mode knob is still on this mode
  }
  // hold at max for a moment
  for (int i = 0; i < 1000; i++) {
    if(getState(pot_1) != state_current) break; // Check if mode knob is still on this mode
  }

  // glow down to min
  for ( userBright; userBright > 0; userBright -= 0.05 ){
    solid(col);
    if(getState(pot_1) != state_current) break; // Check if mode knob is still on this mode
  }

}



/**
 * Current-Limiting code
 * As it stands, if the user manually drives the LED strip, there exists the ability to drive the strip to ~1.5 A.
 * The LED strip is powered from the Vin pin, which can supply only 1.0 A.
 * The following code serves as wrappers around Adafruit's NeoPixel function calls that scales the LED values used
 * to come in under this current limit.
 *
 */

// Wrapper for safe pixel updating
void setPixel(int ledIndex, uint32_t colour){
  ledBuffer[ledIndex] = colour;
}

// Wrapper for safe pixel updating
void update(){
uint8_t R, G, B;

  const float iLim = 0.87; // [A] Current limit (0.9A) for external power supply
  // const float iLim = 0.35; // [A] Current limit for PC USB port
  // const float iLim = 10; // DISABLE current limit
  const float FSDcurrentCorrection = 0.8824; // "Full-scale deflection" correction. The LED response is nonlinear i.e. Amp/LeastSignificantBit is not a constant. This is an attempt to allow the user to specify maximum current as a real value.
  float lsbToAmp = 5.06e-5; // [LSB/Ampere] the relationship between an LED setting and current
  float sum = 0; // Initial sum of currents


  // Sum the LED currents
  for(uint8_t i=0; i<strip.numPixels(); i++) {
    uint32_t temp = ledBuffer[i];
    // Separate the 32bit colour into 8bit R,G,B and add
    // B = ledBuffer[i] & 0xFF;
    // G = (ledBuffer[i] >> 8) & 0xFF;
    // R = (ledBuffer[i] >> 16) & 0xFF;
    B = temp & 0xFF;
    G = (temp >> 8) & 0xFF;
    R = (temp >> 16) & 0xFF;

    sum += float(R + G + B) * lsbToAmp; // Add LED[i]'s current
  }
  sum = sum * FSDcurrentCorrection;
  float scale = float(iLim)/float(sum);


  if ( sum > iLim ) { // Too much current requested
    for(uint8_t i=0; i<strip.numPixels(); i++) {
      uint32_t temp = ledBuffer[i];
      // Separate the 32bit colour into 8bit R,G,B and add
      B = temp & 0xFF;
      G = (temp >> 8) & 0xFF;
      R = (temp >> 16) & 0xFF;

      R = floor(R * scale);
      G = floor(G * scale);
      B = floor(B * scale);

      strip.setPixelColor(i, R, G, B);
    }
  } else {
    for(uint8_t i=0; i<strip.numPixels(); i++) {
      uint32_t temp = ledBuffer[i];
      // Separate the 32bit colour into 8bit R,G,B and add
      B = temp & 0xFF;
      G = (temp >> 8) & 0xFF;
      R = (temp >> 16) & 0xFF;

      strip.setPixelColor(i, R, G, B);
    }
  }


  strip.show();
}
