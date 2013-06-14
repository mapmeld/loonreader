/* 
  LoonReader.pde

  Currently has:
  - startup screen
  - sample text
  - word wrap

  Adapted from Adafruit's open source TFTLCD library
  Please read their comments below to ensure your TFTLCD library is ready to talk to the
  hardware on your Arduino!
*/

// IMPORTANT: Adafruit_TFTLCD library must be specifically
// configured for either the TFT Shield or the Breakout Board.
// See relevant comments in Adafruit_TFTLCD.h for setup.

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library

// The control pins for the LCD can be assigned to any digital or
// analog pins...but using the analog pins allows Arduino to
// also use them with the touch screen
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

void setup(void) {
  Serial.begin(9600);
  progmemPrintln(PSTR("TFT LCD test"));

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  progmemPrintln(PSTR("Using Adafruit 2.8\" TFT Arduino Shield Pinout"));
#else
  progmemPrintln(PSTR("Using Adafruit 2.8\" TFT Breakout Board Pinout"));
#endif

  tft.reset();

  uint16_t identifier = tft.readID();

  if(identifier == 0x9325) {
    progmemPrintln(PSTR("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    progmemPrintln(PSTR("Found ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {
    progmemPrintln(PSTR("Found HX8347G LCD driver"));
  } else {
    progmemPrint(PSTR("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    progmemPrintln(PSTR("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    progmemPrintln(PSTR("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    progmemPrintln(PSTR("should appear in the library header (Adafruit_TFT.h)."));
    progmemPrintln(PSTR("If using the breakout board, it should NOT be #defined!"));
    progmemPrintln(PSTR("Also if using the breakout, double-check that all wiring"));
    progmemPrintln(PSTR("matches the tutorial."));
    return;
  }

  tft.begin(identifier);

  // startup screen for 2.8 seconds
  startText();
  delay(2800);

  // setRotation(1) for landscape mode
  tft.setRotation(1);
  
  // show text
  bookText();

}

void loop(void) {

}

void startText() {
  tft.fillScreen(BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.println("LoonReader");
  tft.println();
  tft.setTextSize(1);
  tft.println("Open Source eBook Reader");
  tft.println("LoonReader.com");
  tft.println("GitHub.com/mapmeld/loonreader");
}

void bookText() {
  tft.fillScreen(BLACK);
  tft.setCursor(8, 0);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  printText("The quick brown fox jumps over the lazy dog. Would you like to know more?");
}

void printText(String text) {
  int i=0;
  int lineLength = 24;
  int space = 32;
  while(i < text.length() ){
    for(int j=i+lineLength;j>=i;j--){
      if((j > text.length() ) || ( text.charAt(j) == space )){
        // insert line breaks, unless we are already at the last line of the text
        String currentLine = "";
        for(int k=i;k<=j;k++){
          currentLine += text.charAt(k);
        }
        tft.println(currentLine);
        i = j + 1;
        break;
      }
      else if(j == i){
        // there were no spaces, print anyway
        String currentLine = "";
        for(int k=i;k<=i+lineLength;k++){
          currentLine += text.charAt(k);
        }
        tft.println(currentLine);
        i += lineLength + 1;
        break;
      }
    }
  }
}

// Copy string from flash to serial port
// Source string MUST be inside a PSTR() declaration!
void progmemPrint(const char *str) {
  char c;
  while(c = pgm_read_byte(str++)) Serial.print(c);
}

// Same as above, with trailing newline
void progmemPrintln(const char *str) {
  progmemPrint(str);
  Serial.println();
}
