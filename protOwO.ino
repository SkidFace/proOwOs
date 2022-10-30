//NOTE TO SELF: I DISABLED GLITCH() DUE TO SRAM POSSIBLY BEING TAKING UP.



#include <ezButton.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"


//Right side of the helmet
static const uint8_t PROGMEM  
  noseL[] =        {     B00000000, 
                        B11111110, 
                        B01111111, 
                        B00000011, 
                        B00000011, 
                        B00000001, 
                        B00000000, 
                        B00000000};
static const uint8_t PROGMEM  
  maw2[] =       {      B00100000, B00000000, 
                        B01111000, B00000000, 
                        B11011110, B00000000, 
                        B11000111, B10000000,
                        B11111111, B11100000, 
                        B00000000, B01111000,
                        B00000000, B00011110, 
                        B00000000, B00000111};
static const uint8_t PROGMEM  
  maw1[] =       {      B00000000, B00000000, 
                        B00000000, B00000000, 
                        B00000000, B00000000, 
                        B00000000, B11100000,
                        B00000111, B11111000, 
                        B00011110, B00011110, 
                        B01111000, B00000111, 
                        B11100000, B00000001};
static const uint8_t PROGMEM  
  maw21[] =       {     B00100000, B00000000, 
                        B01111100, B00000000, 
                        B11011111, B10000000, 
                        B11000111, B11110000,
                        B11111111, B11111100, 
                        B00000001, B11111111,
                        B00000000, B00111111, 
                        B00000000, B00001111};
static const uint8_t PROGMEM  
  maw11[] =       {     B00000000, B00000000, 
                        B00000000, B00000000, 
                        B00000001, B11100000, 
                        B00000111, B11111000,
                        B00111111, B11111100, 
                        B11111111, B00111110, 
                        B11111100, B00001111, 
                        B11110000, B00000011};
static const uint8_t PROGMEM  
  Glitch1_2[] =  {      B00001100, B00000000, B11010100, B10001001, B00010100, B01000111, B11010001, B10100101,
                        B01110100, B00000000, B01001011, B10010110, B00010010, B00000000, B00000000, B00000000};
static const uint8_t PROGMEM  
  Glitch1_1[] =  {      B00101001, B10101001, B01001011, B00101100, B01110100, B00000000, B00000000, B00000000,
                        B00000000, B00000000, B10010101, B10011010, B10010111, B11101010, B10111010, B11010000};
static const uint8_t PROGMEM  
  Glitch2_2[] =  {      B00000000, B00000000, B00000000, B11010101, B10000101, B11101010, B00010111, B00000100,
                        B10100010, B01010100, B10000110, B00010010, B00000101, B00000000, B00000000, B00000000};
static const uint8_t PROGMEM  
  Glitch2_1[] =  {      B01001101, B10011010, B01101001, B10100000, B00001000, B00000000, B00000000, B00000000,
                        B00000000, B00000000, B10001000, B10101010, B00011011, B10010101, B00000001, B00000000};
static const uint8_t PROGMEM  
  Eye[] =        {      B00001111, B00000000, 
                        B00111111, B11100000, 
                        B01111111, B11111000, 
                        B11111111, B11111110,
                        B11110000, B00000111, 
                        B01100000, B00000001, 
                        B00000000, B00000000, 
                        B00000000, B00000000}; 
static const uint8_t PROGMEM  
  Angry[] =      {      B00000000, B00000000,
                        B00011111, B11111100,
                        B00111111, B11111110, 
                        B00111111, B11111100, 
                        B00011111, B11111000,
                        B00001111, B11100000, 
                        B00000011, B10000000, 
                        B00000000, B00000000};                    
static const uint8_t PROGMEM  
  Spooked[] =    {      B00000011, B11000000,
                        B00000111, B11100000,
                        B00001111, B11110000,
                        B00001111, B11110000,
                        B00001111, B11110000,
                        B00001111, B11110000,
                        B00000111, B11100000,
                        B00000011, B11000000};
static const uint8_t PROGMEM  
  vwv[] =        {      B00001110, B00000000, 
                        B00000111, B10000000, 
                        B00000001, B11100000, 
                        B00000000, B01111000, 
                        B00000000, B01111000, 
                        B00000001, B11100000, 
                        B00000111, B10000000, 
                        B00001110, B00000000};
static const uint8_t PROGMEM  
  heart[] =     {       B00000110, B01100000, 
                        B00001111, B11110000, 
                        B00011111, B11111000, 
                        B00011111, B11111000, 
                        B00001111, B11110000, 
                        B00000111, B11100000, 
                        B00000011, B11000000, 
                        B00000001, B10000000};   
                           
//Left side of the helmet
static const uint8_t PROGMEM  
  nose[] =      {       B00000000, 
                        B01111111, 
                        B11111110, 
                        B11000000, 
                        B11000000, 
                        B10000000, 
                        B00000000, 
                        B00000000};
static const uint8_t PROGMEM  
  mawL1[] =      {      B00000000, B00000000, 
                        B00000000, B00000000, 
                        B00000000, B00000000, 
                        B00000111, B00000000, 
                        B00011111, B11100000, 
                        B01111000, B01111000, 
                        B11100000, B00011110, 
                        B10000000, B00000111};
static const uint8_t PROGMEM  
  mawL2[]        {      B00000000, B00000100, 
                        B00000000, B00011110, 
                        B00000000, B01111011, 
                        B00000001, B11100011,
                        B00000111, B11111111, 
                        B00011110, B00000000,
                        B01111000, B00000000, 
                        B11100000, B00000000};
static const uint8_t PROGMEM  
  mawL11[] =     {      B00000000, B00000000, 
                        B00000000, B00000000, 
                        B00000111, B10000000, 
                        B00011111, B11100000, 
                        B00111111, B11111100, 
                        B01111100, B11111111, 
                        B11110000, B00111111, 
                        B11000000, B00001111};
static const uint8_t PROGMEM  
  mawL21[]       {      B00000000, B00000100, 
                        B00000000, B00111110, 
                        B00000001, B11111011, 
                        B00001111, B11100011,
                        B00111111, B11111111, 
                        B11111111, B10000000,
                        B11111100, B00000000, 
                        B11110000, B00000000};
static const uint8_t PROGMEM  
  Glitch1L_1[] = {      B00000000, B00000000, B10101001, B01011001, B11101001, B01010111, B01011101, B00001011,
                        B00000000, B00000000, B00000000, B00000000, B00010010, B10010110, B01001011, B01110100};
static const uint8_t PROGMEM  
  Glitch1L_2[] = {      B10100101, B11010001, B01000111, B00001010, B10001001, B11010100, B00001100, B00000000,
                        B00000000, B00000000, B00000000, B01110100, B00101100, B01001011, B10101001, B00101001};
static const uint8_t PROGMEM  
  Glitch2L_1[] = {      B00000000, B00000000, B00010001, B00000110, B01010101, B11011000, B10101001, B10000000,
                        B00000000, B00000000, B00000000, B00001000, B10100000, B01101001, B10011010, B01001101};
static const uint8_t PROGMEM  
  Glitch2L_2[] = {      B00000000, B00000000, B00000000, B00000101, B00010010, B10000110, B01010100, B10100010,
                        B00000000, B00000000, B00000000, B10101011, B10100001, B01010111, B11101000, B00100000};
static const uint8_t PROGMEM  
  EyeL[] =       {      B00000000, B11110000, 
                        B00000111, B11111100, 
                        B00011111, B11111110, 
                        B01111111, B11111111,
                        B11100000, B00001111, 
                        B10000000, B00000110, 
                        B00000000, B00000000, 
                        B00000000, B00000000};                     
static const uint8_t PROGMEM  
  AngryL[] =     {      B00000000, B00000000, 
                        B00111111, B11111000, 
                        B01111111, B11111100, 
                        B00111111, B11111100,
                        B00011111, B11111000, 
                        B00000111, B11110000, 
                        B00000001, B11000000, 
                        B00000000, B00000000};
static const uint8_t PROGMEM  
  SpookedL[] =   {      B00000011, B11000000,
                        B00000111, B11100000,
                        B00001111, B11110000,
                        B00001111, B11110000,
                        B00001111, B11110000,
                        B00001111, B11110000,
                        B00000111, B11100000,
                        B00000011, B11000000};
static const uint8_t PROGMEM  
  vwvL[] =       {      B00000000, B01110000, 
                        B00000001, B11100000, 
                        B00000111, B10000000, 
                        B00011110, B00000000, 
                        B00011110, B00000000, 
                        B00000111, B10000000, 
                        B00000001, B11100000, 
                        B00000000, B01110000};
static const uint8_t PROGMEM  
  heartL[] =     {      B00000110, B01100000, 
                        B00001111, B11110000, 
                        B00011111, B11111000, 
                        B00011111, B11111000, 
                        B00001111, B11110000, 
                        B00000111, B11100000, 
                        B00000011, B11000000, 
                        B00000001, B10000000};     
                   


Adafruit_8x16matrix matrix4L = Adafruit_8x16matrix();
Adafruit_8x16matrix matrix3L = Adafruit_8x16matrix();
Adafruit_8x16matrix matrix2L = Adafruit_8x16matrix();
Adafruit_8x8matrix  matrix1L = Adafruit_8x8matrix();
Adafruit_8x8matrix  matrix1R = Adafruit_8x8matrix();
Adafruit_8x16matrix matrix2R = Adafruit_8x16matrix();
Adafruit_8x16matrix matrix3R = Adafruit_8x16matrix();
Adafruit_8x16matrix matrix4R = Adafruit_8x16matrix();

int expressions = 4; // Here you can change the amount of expressions you want to use

const int BUTTON_PIN = 2; 
const int IR_TOGGLE = 3; 
ezButton faceSwap(BUTTON_PIN);
ezButton irToggle(IR_TOGGLE);
bool irEnabled = true;
const int interruptPin2 = A1;
//volatile long currentTime = 0;
volatile long debounceTime2 = 0;
volatile long debounceTime3 = 0;
volatile long previousMillis = 0;
const long interval = 1000;

int y;
int x;
int Step = -7;
int redPin = 5; 
int greenPin = 9;
int bluePin = 10;
int counter = 0;
byte state = 0;
byte state2;
bool rising = 1;
int IrPin = A7;
int Ir = 0;
bool blinkstate = 0;
unsigned long blinkDelay = 9000;

void setup() {
  //attachInterrupt(digitalPinToInterrupt(2), ISR_button, RISING);
  matrix4L.begin(0x77);   
  matrix3L.begin(0x76);   
  matrix2L.begin(0x75);
  matrix1L.begin(0x73);
  matrix1R.begin(0x74);
  matrix2R.begin(0x72);
  matrix3R.begin(0x71);
  matrix4R.begin(0x70);
  
  matrix4L.setRotation(1);   
  matrix3L.setRotation(3);
  matrix2L.setRotation(1);
  matrix1L.setRotation(3);
  matrix1R.setRotation(1);
  matrix2R.setRotation(3);
  matrix3R.setRotation(1);
  matrix4R.setRotation(1);
  ClearDisplay();

  Serial.begin(57600);
  faceSwap.setDebounceTime(50); 
  irToggle.setDebounceTime(50); 

#ifndef visorSensor
  state2 = 1;
#endif

#ifdef visorSensor
  pinMode(interruptPin2, INPUT);
//  attachInterrupt(digitalPinToInterrupt(3), hallSensor, CHANGE);
  if((PIND & B00001000) == 0){
    state2 = 1;
  }
#endif


}


///////////////////////////Actual code begins here\\\\\\\\\\\\\\\\\\\\\\\\\\\

void loop() {
  faceSwap.loop();
  irToggle.loop();
  if(faceSwap.isPressed()) {
    Serial.println("boop");
    ChangeFace();
  }
  if(irToggle.isPressed()) {
    irEnabled = !irEnabled;
    Serial.println("PAWg");
  }
  if (state2 == 1) {
    if (Serial.available() > 0) {
      if(Serial.read() != state){
        state = Serial.read();
        Serial.println(state);
        rising = 1;
      }
    }
    Ir = analogRead(IrPin);
    if (Ir >= 500) {
      if (rising == 1) {
        ClearDisplay();
        matrix3L.drawBitmap(0, 0, mawL2, 16, 8, HIGH);
        matrix2L.drawBitmap(0, 0, mawL1, 16, 8, HIGH);
        matrix1L.drawBitmap(0, 0, noseL, 8, 8, HIGH);
        matrix1R.drawBitmap(0, 0, nose, 8, 8, HIGH);
        matrix2R.drawBitmap(0, 0, maw1, 16, 8, HIGH);
        matrix3R.drawBitmap(0, 0, maw2, 16, 8, HIGH);
        //Serial.println(state);  
        switch (state) {
          case 0:                             //First button press: Happy expression
            matrix4L.drawBitmap(0, 0, EyeL, 16, 8, HIGH);
            matrix4R.drawBitmap(0, 0, Eye, 16, 8, HIGH);
            rising = 0;
            if (blinkstate == 0) {
              setColor(0, 30, 0);               //Makes the colour of the rgb LED green
            }
            previousMillis = millis();
            break;

          case 1:                             //Second button press: Surprised
              matrix4L.drawBitmap(0, 0, SpookedL, 16, 8, HIGH);
              matrix4R.drawBitmap(0, 0, Spooked, 16, 8, HIGH);

            rising = 0;
            if (blinkstate == 0) {
              setColor(0, 0, 30);              //Makes the colour of the LED blue
            }
            previousMillis = millis();
            break;

          case 2:                             //Third button press: Angry expression
            matrix4L.drawBitmap(0, 0, AngryL, 16, 8, HIGH);
            matrix4R.drawBitmap(0, 0, Angry, 16, 8, HIGH);
            rising = 0;
            if (blinkstate == 0) {
              setColor(30, 0, 0);              //Makes the colour of the LED red
            }
            previousMillis = millis();
            break;

          case 3:
            matrix4L.drawBitmap(0, 0, vwvL, 16, 8, HIGH);
            matrix4R.drawBitmap(0, 0, vwv, 16, 8, HIGH);
            rising = 0;
            if (blinkstate == 0) {
              setColor(60, 30, 0);                //Makes the colour of the LED orange
            }
            previousMillis = millis();
            break;
          case 4:
            matrix4L.drawBitmap(0, 0, heartL, 16, 8, HIGH);
            matrix4R.drawBitmap(0, 0, heart, 16, 8, HIGH);
            rising = 0;
            if (blinkstate == 0) {
              setColor(60, 0, 60);                //Makes the colour of the LED orange
            }
            previousMillis = millis();
            break;
        }
        blinkstate = 0;
      }
      //glitch();
      Blink();
      writeDisplay();
      if (millis() - previousMillis >= interval) {
        setColor(0, 0, 0);
      }
      
    }
    else {
      if(irEnabled){
        debounceTime2 = millis();
        matrix4L.fillRect(0, 0, 16, 8, LOW);
        matrix4R.fillRect(0, 0, 16, 8, LOW);
        matrix4L.drawBitmap(0, 0, vwvL, 16, 8, HIGH);
        matrix4R.drawBitmap(0, 0, vwv, 16, 8, HIGH);
        setColor(60, 30, 0);
        rising = 1;
        writeDisplay();
      }
    }
  }
  else {
    setColor(0, 0, 0); 
    ClearDisplay();
    writeDisplay();
  }
}
///////////////////////////LED\\\\\\\\\\\\\\\\\\\\\\\\\\\

void setColor(int redValue, int greenValue, int blueValue) {
  redValue = redValue;            //  Remove the slashes here 
  greenValue = greenValue;        //  if the status led is not 
  blueValue = blueValue;          //  lighting up properly
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

///////////////////////////Glitch animation\\\\\\\\\\\\\\\\\\\\\\\\\\\

//void glitch() {
//  if ((millis() - debounceTime3) > 7000) {
//    if (state == 2) {
//      counter++;
//      if (counter == 1) {
//        matrix3L.fillRect(0, 0, 16, 8, LOW);
//        matrix2L.fillRect(0, 0, 16, 8, LOW);
//        matrix2R.fillRect(0, 0, 16, 8, LOW);
//        matrix3R.fillRect(0, 0, 16, 8, LOW);
//        matrix2L.drawBitmap(0, 0, Glitch1L_1, 16, 8, HIGH);
//        matrix3L.drawBitmap(0, 0, Glitch1L_2, 16, 8, HIGH);
//        matrix2R.drawBitmap(0, 0, Glitch1_1, 16, 8, HIGH);
//        matrix3R.drawBitmap(0, 0, Glitch1L_2, 16, 8, HIGH);
//        writeDisplay();
//        delay(170);
//        counter++;
//      }
//      if (counter == 2) {
//        matrix3L.fillRect(0, 0, 16, 8, LOW);
//        matrix2L.fillRect(0, 0, 16, 8, LOW);
//        matrix2R.fillRect(0, 0, 16, 8, LOW);
//        matrix3R.fillRect(0, 0, 16, 8, LOW);
//        matrix2L.drawBitmap(0, 0, Glitch2L_1, 16, 8, HIGH);
//        matrix3L.drawBitmap(0, 0, Glitch2L_2, 16, 8, HIGH);
//        matrix2R.drawBitmap(0, 0, Glitch2_1, 16, 8, HIGH);
//        matrix3R.drawBitmap(0, 0, Glitch2_2, 16, 8, HIGH);
//        writeDisplay();
//        delay(200);
//        counter++;
//      }
//      if (counter == 3) {
//        matrix3L.fillRect(0, 0, 16, 8, LOW);
//        matrix2L.fillRect(0, 0, 16, 8, LOW);
//        matrix2R.fillRect(0, 0, 16, 8, LOW);
//        matrix3R.fillRect(0, 0, 16, 8, LOW);
//        matrix2L.drawBitmap(0, 0, mawL1, 16, 8, HIGH);
//        matrix3L.drawBitmap(0, 0, mawL2, 16, 8, HIGH);
//        matrix2R.drawBitmap(0, 0, maw1, 16, 8, HIGH);
//        matrix3R.drawBitmap(0, 0, maw2, 16, 8, HIGH);
//        writeDisplay();
//        counter = 0;
//      }
//      debounceTime3 = millis();
//    }
//  }
//}

///////////////////////////Eye-tracking\\\\\\\\\\\\\\\\\\\\\\\\\\\


///////////////////////////Blink animation\\\\\\\\\\\\\\\\\\\\\\\\\\\

void Blink() {
  if ((millis() - debounceTime2) > blinkDelay)  {                    //The blinking animation begins here

    for (Step > 0; Step++;) {
      matrix4L.fillRect(0, Step, 16, 8, LOW);
      matrix4R.fillRect(0, Step, 16, 8, LOW);
      writeDisplay();
      delay(25);
    }
    debounceTime2 = millis();
  }
  if (Step != -7) {
    matrix4L.fillRect(0, Step, 16, 8, LOW);
    matrix4R.fillRect(0, Step, 16, 8, LOW);
    Step--;
    delay(25);
    rising = 1;
    blinkstate = 1;
  }
}

///////////////////////////Matrix writing icons\\\\\\\\\\\\\\\\\\\\\\\\\\\

void writeDisplay() {
  matrix4L.writeDisplay();
  matrix3L.writeDisplay();
  matrix2L.writeDisplay();
  matrix1L.writeDisplay();
  matrix1R.writeDisplay();
  matrix2R.writeDisplay();
  matrix3R.writeDisplay();
  matrix4R.writeDisplay();
}

void ClearDisplay(){
  matrix4L.fillScreen(LOW);
  matrix3L.fillScreen(LOW);
  matrix2L.fillScreen(LOW);
  matrix1L.fillScreen(LOW);
  matrix1R.fillScreen(LOW);
  matrix2R.fillScreen(LOW);
  matrix3R.fillScreen(LOW);
  matrix4R.fillScreen(LOW);
}

void ChangeFace() {                               //Stuff you shouldn't touch :P
    if (state < expressions) {
      state++;
    }
    else {
      state = 0;
    }
    rising = 1;
}

//void ISR_button() {                               //Stuff you shouldn't touch :P
//  currentTime = millis();
//  if ((currentTime - debounceTime) > 250) {
//    if (state < expressions) {
//      state++;
//    }
//    else {
//      state = 0;
//    }
//    rising = 1;
//  }
//  debounceTime = currentTime;
//}
