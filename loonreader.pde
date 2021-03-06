/* 
  LoonReader.pde

  Currently has:
  - startup screen
  - sample text
  - reading text from SD card
  - word wrap
  - pagination
  - touch to see next page

  Adapted from Adafruit's open source TFTLCD library
  Please read their comments below to ensure your TFTLCD library is ready to talk to the
  hardware on your Arduino!
*/

// IMPORTANT: Adafruit_TFTLCD library must be specifically
// configured for either the TFT Shield or the Breakout Board.
// See relevant comments in Adafruit_TFTLCD.h for setup.

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <stdint.h>
#include <TouchScreen.h>
#include <SD.h>

// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// This demo code returns raw readings, public domain

// These are the pins for the shield!
#define YP A1  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 7   // can be a digital pin
#define XP 6   // can be a digital pin

#define MINPRESSURE 10
#define MAXPRESSURE 1000

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

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

#define SD_CS 5 // Card select for shield use
uint8_t         spi_save;
File root;
File bookFile;

int page = 0;
int lineLength = 24;
int lineCount = 10;
String progressWord;

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

  // SD Card init
  if (!SD.begin(SD_CS)) {
    return;
  }
  spi_save = SPCR;

    // open first file for reading:
    //root = SD.open("/");
    //root.rewindDirectory();
    bookFile = SD.open("BOOK.TXT");
    progressWord = "" + char( bookFile.read() );
    //root.openNextFile();
    //root.close();
    
    // read from the file until there's nothing else in it:
    //myBook = "";
    //while (bookFile.available() > 3976 && myBook.length() < lineLength * lineCount) {
      //myBook += char( bookFile.read() );
    //}
    // close the file:
    //bookFile.close();

  // reset pins so you can use touch screen
  spi_save = 0;
  SPCR = 0;

  // startup screen for 2.8 seconds
  startText();
  delay(2800);
  
  pinMode(13, OUTPUT);
}

void loop(void) {

  if(page == 0){
    // setRotation(1) for landscape mode
    tft.setRotation(1);
 
    // show text
    page = 1;
    bookText(1);
  }
  
  // a point object holds x y and z coordinates
  digitalWrite(13, HIGH);
  Point p = ts.getPoint();
  digitalWrite(13, LOW);

  // if sharing pins, you'll need to fix the directions of the touchscreen pins
  //pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  //pinMode(YM, OUTPUT);
  
  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
     Serial.print("X = "); Serial.print(p.x);
     Serial.print("\tY = "); Serial.print(p.y);
     Serial.print("\tPressure = "); Serial.println(p.z);
     
     // registering a touch
     page++;
     bookText(page);
  }
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

void bookText(int loadpage) {
  tft.fillScreen(BLACK);
  tft.setCursor(8, 0);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  if(loadpage <= 1){
    printText("The quick brown fox jumps over the lazy dog. Would you like to know more?");
  }
  else{
    printTextFromFile();
  }
}

void printTextFromFile() {
  int printedLines = 0;
  String progressLine = "";
  int space = 32;
  //spi_save = SPCR;
  
  //Serial.println( bookFile.available() );
  
  while ( bookFile.available() && printedLines < lineCount ){
    // continuously add chars
    
    char readChar = bookFile.read();
        
    progressWord += char( readChar );
    if(readChar == space){
      // hit a space, add it to the line
      progressLine += progressWord;
      progressWord = "";
    }
    if(progressWord.length() >= lineLength){
      // no spaces, already on new line
      //spi_save = 0;
      //SPCR = 0;
      tft.println( progressWord );
      progressWord = "";
      printedLines++;
    }
    if(progressLine.length() + progressWord.length() >= lineLength){
      // going to need a new line, print old progressLine now
      //spi_save = 0;
      //SPCR = 0;
      tft.println( progressLine );
      progressLine = "";
      printedLines++;
    }
    //spi_save = SPCR;
  }
  if(printedLines < lineCount && (progressLine.length() > 0 || progressWord.length() > 0 )){
    //spi_save = 0;
    //SPCR = 0;
    tft.println( progressLine + progressWord );
    progressLine = "";
    progressWord = "";
  }
}

void printText(String text) {
  int i=0;
  int space = 32;
  int printedLines = 0;
  while(i < text.length() && printedLines < lineCount ){
    for(int j=i+lineLength;j>=i;j--){
      if((j > text.length() ) || ( text.charAt(j) == space )){
        // insert line breaks, unless we are already at the last line of the text
        String currentLine = "";
        for(int k=i;k<=j;k++){
          currentLine += text.charAt(k);
        }
        tft.println(currentLine);
        printedLines++;
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
        printedLines++;
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