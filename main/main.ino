#include <arduinoFFT.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <dht.h>
#include <virtuabotixRTC.h>

#define NEXT_BUTTON 2
#define DHT_SENSOR 3
#define MAX_MODES 7
#define LDR A3
#define MATRIX_CS 10
#define DISPLAY_NUMBER 4

MD_MAX72XX disp = MD_MAX72XX(MD_MAX72XX::FC16_HW, MATRIX_CS, DISPLAY_NUMBER);
MD_Parola myDisplay = MD_Parola(MD_MAX72XX::FC16_HW, MATRIX_CS, DISPLAY_NUMBER);
arduinoFFT FFT = arduinoFFT();
dht DHT;
virtuabotixRTC myRTC(6, 7, 8);

volatile int mode = 0;        // Current mode
volatile unsigned long lastInt = 0; // Debouncer time variable
int LDR_value = 0; // Intiial value of the LDR resistor


double realComponent[64];
double imagComponent[64];


int spectralHeight[] = {0b00000000,0b10000000,0b11000000,


                        0b11100000,0b11110000,0b11111000,


                        0b11111100,0b11111110,0b11111111};

// Sprite definitions:
const uint8_t F_PMAN2 = 6;
const uint8_t W_PMAN2 = 18;
const uint8_t PROGMEM pacman2[F_PMAN2 * W_PMAN2] =  // pacman pursued by a ghost
{
  0x00, 0x81, 0xc3, 0xe7, 0xff, 0x7e, 0x7e, 0x3c, 0x00, 0x00, 0x00, 0xfe, 0x7b, 0xf3, 0x7f, 0xfb, 0x73, 0xfe,
  0x00, 0x42, 0xe7, 0xe7, 0xff, 0xff, 0x7e, 0x3c, 0x00, 0x00, 0x00, 0xfe, 0x7b, 0xf3, 0x7f, 0xfb, 0x73, 0xfe,
  0x24, 0x66, 0xe7, 0xff, 0xff, 0xff, 0x7e, 0x3c, 0x00, 0x00, 0x00, 0xfe, 0x7b, 0xf3, 0x7f, 0xfb, 0x73, 0xfe,
  0x3c, 0x7e, 0xff, 0xff, 0xff, 0xff, 0x7e, 0x3c, 0x00, 0x00, 0x00, 0xfe, 0x7b, 0xf3, 0x7f, 0xfb, 0x73, 0xfe,
  0x24, 0x66, 0xe7, 0xff, 0xff, 0xff, 0x7e, 0x3c, 0x00, 0x00, 0x00, 0xfe, 0x7b, 0xf3, 0x7f, 0xfb, 0x73, 0xfe,
  0x00, 0x42, 0xe7, 0xe7, 0xff, 0xff, 0x7e, 0x3c, 0x00, 0x00, 0x00, 0xfe, 0x7b, 0xf3, 0x7f, 0xfb, 0x73, 0xfe,
};

int index, c, value;

void change() {
  unsigned long currentMillis = millis();
  
  // Debounce the button
  if (currentMillis - lastInt > 300) {
    // Update the mode
    mode += 1;
    if (mode >= MAX_MODES)
      mode = 0;

    // Special settings for modes
    if (mode == 0) {
      myDisplay.setSpriteData(pacman2, W_PMAN2, F_PMAN2, pacman2, W_PMAN2, F_PMAN2);
      myDisplay.displayText("Hello", PA_CENTER, 50, 1000, PA_SPRITE, PA_SPRITE);
    }
    if (mode == 6) {
      myDisplay.displayScroll("M-a cumparat pe 200 de lei, nu-l credeti!", PA_CENTER, PA_SCROLL_LEFT, 100);
    }

    lastInt = currentMillis;
  }
}

void setup() {
  pinMode(NEXT_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(NEXT_BUTTON), change, FALLING);

  disp.begin();
  myDisplay.begin();
  myDisplay.setIntensity(0);
  myDisplay.displayClear();
  Serial.begin(9600);
  // myRTC.setDS1302Time(10, 56, 2, 2, 30, 5, 2023);

  myDisplay.setSpriteData(pacman2, W_PMAN2, F_PMAN2, pacman2, W_PMAN2, F_PMAN2);
  myDisplay.displayText("Hello", PA_CENTER, 50, 1000, PA_SPRITE, PA_SPRITE);
}

void loop() {
  // Read data from the LDR for light intensity of the matrix
  float alpha = 0.2;  // Smoothing factor
  int rawValue = analogRead(LDR);
  LDR_value = alpha * rawValue + (1 - alpha) * LDR_value; // reduce noise

  // map the values to a single digit 
  int intensity = LDR_value / 100;

  myDisplay.setIntensity(intensity);

  if (mode == 0) {
    // Animate the matrix prints using the set options in change
    if (myDisplay.displayAnimate()) {
      myDisplay.displayReset();
    }
  } else if (mode == 1) {
    // Set a sensisitivity
    int sensitivity = map(analogRead(A0),0,1023,50,100); 

    // Get info and normalize based on senstivity
    for(int i=0; i<64; i++) {
      realComponent[i] = analogRead(A0)/sensitivity;
      imagComponent[i] = 0;
    }

    // Apply Hamming windowing function to reduce spectral leakage
    FFT.Windowing(realComponent, 64, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    // Perform the actual FFT calculations
    FFT.Compute(realComponent, imagComponent, 64, FFT_FORWARD);
    // convert from FFT resutls to magnitudes (amplitudes of the frequencies)
    FFT.ComplexToMagnitude(realComponent, imagComponent, 64);

    for(int i=0; i<32; i++) {
      // Constrain the values up to 80 for reasonable output 
      realComponent[i] = constrain(realComponent[i],0,80);
      // Map the values up to 8 for the 32x8 (8 led size column) matrix 
      realComponent[i] = map(realComponent[i],0,80,0,8);

      // Store an get the proper spectral amplitude
      index = realComponent[i];
      value = spectralHeight[index];

      // get the column position
      c = 31 - i;

      // Set the leds values
      disp.setColumn(c, value);
    }
  } else if (mode == 2) {
    // Read data from DHT11 sensor connected to the specified pin
    int readData = DHT.read11(DHT_SENSOR);

    // Read the temperature value from the DHT sensor
    float t = DHT.temperature;

    // Set the text alignment to center on the display
    myDisplay.setTextAlignment(PA_CENTER);

    // Round the temperature value and display it on the display as a string followed by "*C"
    int t_rounded = round(t);
    myDisplay.print((String)t_rounded+"*C");
  } else if (mode == 3) {
    // Read data from DHT11 sensor connected to the specified pin
    int readData = DHT.read11(DHT_SENSOR);

    // Read the humidity value from the DHT sensor
    float h = DHT.humidity;

    // Set the text alignment to center on the display
    myDisplay.setTextAlignment(PA_CENTER);

    // Round the humidity value and display it on the display as a string followed by "%"
    int h_rounded = round(h);
    myDisplay.print((String)h_rounded+"%");
  } else if (mode == 4) {
    // Update the time in the RTC module
    myRTC.updateTime();

    // Retrieve the hours and minutes from the RTC module
    int hours = myRTC.hours;
    int minutes = myRTC.minutes;

    // Set the text alignment to center on the display
    myDisplay.setTextAlignment(PA_CENTER);

    // Create a string representing the time in "hours:minutes" format
    String time = (String)hours + ":" + (String)minutes;

    // Add leading zero to minutes if it is a single digit number
    if (minutes >= 0 && minutes <= 9) {
      time = (String)hours + ":" + "0" + (String)minutes;
    }

    // Display the formatted time on the display
    myDisplay.print(time);
  } else if (mode == 5) {
    // Prin the text with a simple center effect
    myDisplay.setTextAlignment(PA_CENTER);
    myDisplay.print("Nota 10");
  } else if (mode == 6) {
    // Animate the matrix prints using the set options in change
    if (myDisplay.displayAnimate()) {
      myDisplay.displayReset();
    }
  }
}
