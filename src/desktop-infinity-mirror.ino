/*
* This program drives the Core Electronics Infinity Kit
* http://coreelec.io/infinitykit
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


/*
 * AUTOMATIC: Photon must have a valid WiFi network to begin running this code.
 * SEMI_AUTOMATIC: Photon will run this code immediately, and attempt to connect to a WiFi network only if SETUP button is pressed.
 */
SYSTEM_MODE(SEMI_AUTOMATIC);



/*******************************************************************************
* Hardware Definitions
* You won't need to change these
******************************************************************************/
const int strip_pin = 0; // The digital pin that drives the LED strip
const int num_leds = 44; // Number of LEDs in the strip. Shouldn't need changing unless you hack the hardware
const int pot = 0; // Potentiometer pin selects the mode
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
  state_edge
};

static uint8_t state; // The state that the user demands
static uint8_t state_current; // The state currently being executed

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
  update(); // Initialize all pixels to off
}



/*******************************************************************************
* LOOP
*
******************************************************************************/
void loop()
{
  static uint32_t userColour = Wheel(0); // User-set colour TODO roll into LED-strip object

  connectWIFIonButtonPress();

  // Read potentiometer values for user-input
  state = getState(pot);    // Select the operation mode

  // State Machine.
  switch(state){
    case state_off:
    clearStrip();   // "Off" state.
    delay(50);
    break;

    case state_rainbow:
    rainbow(); // Adafruit's rainbow demo, modified for seamless wraparound. We are passing the Pot # instead of the option because delay value needs to be updated WITHIN the rainbow function. Not just at the start of each main loop.
    break;

    case state_brightness:
    brightness(userColour);
    break;

    case state_comet:
    comet(userColour); // An under-construction comet demo.
    break;

    case state_solid:
    solid(userColour);
    break;

    case state_scroll:
    userColour = scroll();
    break;

    case state_edge:
    edge(userColour);
    break;

    default: // If getState() returns some unexpected value, this section will execute
    break;

  }
}


/*******************************************************************************
* Functions
*
******************************************************************************/
/*
*  Connect to stored WiFi credentials. Only useful if you have claimed your
*  particle photon to your particle account: https://build.particle.io
*/
void connectWIFIonButtonPress() {
  if (System.buttonPushed() > 1) {
    if( !Particle.connected() ) Particle.connect();
  }
}

// Break potentiometer rotation into sectors for setting mode
// This is the function that determines the order that settings are available from the user-input pot.
uint8_t getState(int pot){
  // TODO: find better, more flexible method of defining pot sectors? Remove magic number?
  float val = float(analogRead(pot)) / float(ADC_precision);

  if (val < 0.05) {
    return state_off;
  } else if (val < 0.25) {
    return state_rainbow;
  }else if (val < 0.5) {
    return state_scroll;
  } else if (val < 0.75) {
    return state_comet;
  } else if (val < 0.95) {
    return state_solid;
  } else {
    // return state_brightness;
    return state_edge;
  }
}



/*******************************************************************************
 * Let's create our own mode!
 *******************************************************************************/
void edge(uint32_t colour){
  state_current = state_edge;
  int edgeLength = strip.numPixels()/4;

  // grow line along 4 edges
  for(int i = 0; i < edgeLength; i++){
    setPixel(i, colour);                // First edge
    setPixel(i +   edgeLength, colour); // second edge
    setPixel(i + 2*edgeLength, colour); // third edge
    setPixel(i + 3*edgeLength, colour); // fourth edge
    update();
    delay(50);
    if(getState(pot) != state_current) break; // Check if mode knob is still on this mode
  }

  // shrink line along 4 edges
  for(int i = 0; i < edgeLength; i++){
    setPixel(i, strip.Color(0,0,0));
    setPixel(i +   edgeLength, strip.Color(0,0,0));
    setPixel(i + 2*edgeLength, strip.Color(0,0,0));
    setPixel(i + 3*edgeLength, strip.Color(0,0,0));
    update();
    delay(50);
    if(getState(pot) != state_current) break; // Check if mode knob is still on this mode
  }



}



/* Run the comet demo
* This feature is largely experimental and quite incomplete.
* The idea is to involve multiple comets that can interact by colour-addition
*/
void comet(uint32_t colour){
  state_current = state_comet;
  uint16_t i, j, k;
  uint16_t ofs = 15;

  for (j=0; j<strip.numPixels(); j++){
    clearStrip();

    drawComet(j, colour);

    update();
    delay(30);
    if(getState(pot) != state_current) break; // Check if mode knob is still on this mode
  }
}


/*
* Draw a comet on the strip and handle wrapping gracefully.
* TODO:
*      - Handle multiple comets
*/
void drawComet(uint16_t pos, uint32_t colour) {
  float headBrightness = 255;                 // Brightness of the first LED in the comet
  uint8_t bright = uint8_t(headBrightness);   // Initialise the brightness variable
  uint16_t len = 20;                          // Length of comet tail
  double lambda = 0.3;                        // Parameter that effects how quickly the comet tail dims
  double dim = lambda;                        // initialise exponential decay function

  // Get colour of comet head, and prepare variables to use for tail-dimming
  uint8_t headR, headG, headB, R, G, B;
  colourToRGB(colour, &headR, &headG, &headB);
  R = headR;
  G = headG;
  B = headB;

  setPixel(pos, colour); // Head of comet

  for(uint16_t i=1; i<len; i++){
    // Figure out if the current pixel is wrapped across the strip ends or not, light that pixel
    if( pos - i < 0 ){ // Wrapped
      setPixel(strip.numPixels()+pos-i, strip.Color(R,G,B));
    } else { // Not wrapped
      setPixel(pos-i, strip.Color(R,G,B));
    }
    R = uint8_t(headR * exp(-dim)); // Exponential decay function to dim tail LEDs
    G = uint8_t(headG * exp(-dim));
    B = uint8_t(headB * exp(-dim));
    dim += lambda;
  }
}


void rainbow() {
  //   uint16_t j;
  float i, baseCol;
  float colStep = 256.0 / strip.numPixels();

  for(baseCol=0; baseCol<256; baseCol++) { // Loop through all colours
    for(i=0; i<strip.numPixels(); i++) {   // Loop through all pixels
      setPixel( i, Wheel(int(i*(colStep)+baseCol) & 255) ); // This line seamlessly wraps the colour around the table.
    }
    update();
    delay(42);

    if(getState(pot) != state_rainbow) break; // Check if mode knob is still on this mode
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


// Display a single colour on all LEDs. While accessible from the state machine, this function does not set/check
// the state variables, because it is called by other states.
void solid(uint32_t colour){
  // Do not set state_current here.
  uint16_t i;
  // User-defined colour
  for(i=0; i<strip.numPixels(); i++){
    setPixel(i, colour);
  }

  update();
  delay(50);
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

  const float maxBright = 1.0; // Leave this as 1. Taking it above 1 won't make things brighter.
  const float minBright = 0.01; // must be strictly greater than zero

  // glow on to max
  for ( userBright; userBright < maxBright; userBright += 0.05*userBright ){
    solid(col);
    if(getState(pot) != state_current) break; // Check if mode knob is still on this mode
  }
  userBright = min(userBright, maxBright); // Prevent overshooting 1.0

  // hold at max for a moment
  for (int i = 0; i < 20; i++) {
    if(getState(pot) != state_current) break; // Check if mode knob is still on this mode
    solid(col);
  }

  // glow down to min
  for ( userBright; userBright > minBright; userBright -= 0.05*userBright ){
    solid(col);
    if(getState(pot) != state_current) break; // Check if mode knob is still on this mode
  }
  userBright = max(minBright, userBright); // Prevent undershoot
  userBright = max(userBright, 0); // Prevent dead-locking at zero, just in case user ignored minBright requirements

  // hold at min for a moment
  for (int i = 0; i < 20; i++) {
    if(getState(pot) != state_current) break; // Check if mode knob is still on this mode
    solid(col);
  }

}




/***************************************************************************************************

8888888b.                                                   8888888888P
888  "Y88b                                                        d88P
888    888                                                       d88P
888    888  8888b.  88888b.   .d88b.   .d88b.  888d888          d88P    .d88b.  88888b.   .d88b.
888    888     "88b 888 "88b d88P"88b d8P  Y8b 888P"           d88P    d88""88b 888 "88b d8P  Y8b
888    888 .d888888 888  888 888  888 88888888 888            d88P     888  888 888  888 88888888
888  .d88P 888  888 888  888 Y88b 888 Y8b.     888           d88P      Y88..88P 888  888 Y8b.
8888888P"  "Y888888 888  888  "Y88888  "Y8888  888          d8888888888 "Y88P"  888  888  "Y8888
                                  888
                            Y8b d88P
                            "Y88P"


Changing the code below this line could REALLY DAMAGE your Infinity Mirror.
***************************************************************************************************/


/**
* Current-Limiting code
* As it stands, if the user manually drives the LED strip, there exists the ability to drive the strip to ~1.5 A.
* The LED strip is powered from the Vin pin, which can supply only 1.0 A.
* The following code serves as wrappers around Adafruit's NeoPixel function calls that scales the user-demanded
* LED values if they would result in LEDs drawing too much current
*
*/

// Wrapper for LED buffer
void setPixel(int ledIndex, uint32_t colour){
  ledBuffer[ledIndex] = colour;
}

// Wrapper for safe pixel updating - prevent user from requesting too much current
// TODO refactor, retain brightness adjusted calculations through the function to avoid re-computing and improve readability
void update(){
  uint8_t R, G, B;

  const float iLim = 0.87; // [A] Current limit (0.9A) for external power supply or 1A capable computer USB port.
  // const float iLim = 0.35; // [A] Current limit for 500mA computer USB port
  // const float iLim = 10; // DANGER effectively DISABLE current limiting.
  const float FSDcurrentCorrection = 0.8824; // "Full-scale deflection" correction. The LED response is nonlinear i.e. Amp/LeastSignificantBit is not a constant. This is an attempt to allow the user to specify maximum current as a real value.
  float lsbToAmp = 5.06e-5; // [LSB/Ampere] the relationship between an LED setting and current
  float sum = 0; // Initial sum of currents


  // Sum the LED currents
  for(uint8_t i=0; i<strip.numPixels(); i++) {
    // Separate the 32bit colour into 8bit R,G,B then scale to the desired brightness
    colourToRGB(ledBuffer[i], &R, &G, &B);
    applyBrightness(&R,&G,&B);

    sum += float(R + G + B) * lsbToAmp; // Add LED[i]'s current
  }
  sum = sum * FSDcurrentCorrection;
  float scale = float(iLim)/float(sum);


  if ( sum > iLim ) { // Too much current requested
    for(uint8_t i=0; i<strip.numPixels(); i++) {
      // Separate the 32bit colour into 8bit R,G,B then scale to the desired brightness
      colourToRGB(ledBuffer[i], &R, &G, &B);
      applyBrightness(&R,&G,&B);

      // Scale down to current limit
      R = floor(R * scale);
      G = floor(G * scale);
      B = floor(B * scale);

      strip.setPixelColor(i, R, G, B);
    }
  } else {
    for(uint8_t i=0; i<strip.numPixels(); i++) {
      // Separate the 32bit colour into 8bit R,G,B then scale to the desired brightness
      colourToRGB(ledBuffer[i], &R, &G, &B);
      applyBrightness(&R,&G,&B);

      strip.setPixelColor(i, R, G, B);
    }
  }

  strip.show();
}

// INPUT:32-bit colour and OUTPUT: 8-bit R,G,B
void colourToRGB(uint32_t col, uint8_t *R, uint8_t *G, uint8_t *B) {
  *B = col & 0xFF;
  *G = (col >> 8) & 0xFF;
  *R = (col >> 16) & 0xFF;
}

// Scale the demanded colour by the user set brightness 0-100%
void applyBrightness(uint8_t *R, uint8_t *G, uint8_t *B) {
  *B = userBright * *B;
  *G = userBright * *G;
  *R = userBright * *R;
}


// Clear all leds and update. Clears LED buffer too.
void clearStrip(void){
  for(uint8_t i=0; i<strip.numPixels(); i++){
    setPixel(i, strip.Color(0,0,0));
  }
  update();
}
