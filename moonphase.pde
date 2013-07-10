
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Time.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 3
// Connect the GPS RX (receive) pin to Digital 2

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

float tz = -4;  // Sets timezone to GMT -4 (US Eastern)

float quadout[5];
float mooneq[3];
char south = 'S';
char west = 'W';

void setup(){
    
  // connect at 115200 so we can read the GPS fast enough
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit GPS
  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop(){
  if (! usingInterrupt) {
    // replace for interrupt: read data from the GPS in the 'main loop'
    char c = GPS.read();
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);

    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);

    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //Serial.print("Angle: "); Serial.println(GPS.angle);
      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      if(GPS.year < 50){
        // right date found - before 2050
        checkSunMoon();
      }
    }
  }
}

float getmjd(int day, int month, int year, float hour){
   float b;
   if (month <= 2) {
     month = month + 12;
     year = year - 1;
   }
   float a = 10000.0 * year + 100.0 * month + day;
   if(a <= 15821004.1){
     b = -2 * floor((year + 4716)/4) - 1179;
   }
   else{
     b = floor(year/400) - floor(year/100) + floor(year/4);
   }
   a = 365.0 * year - 679004.0;
   return (a + b + floor(30.6001 * (month + 1)) + day + hour/24.0);
}

void quad(float ym, float yz, float yp) {
  float nz, a, b, c, dis, dx, xe, ye, z1, z2;
  nz = 0;
  a = 0.5 * (ym + yp) - yz;
  b = 0.5 * (yp - ym);
  c = yz;
  xe = -b / (2.0 * a);
  ye = (a * xe + b) * xe + c;
  dis = b * b - 4.0 * a * c;
  if (dis > 0)	{
    dx = 0.5 * sqrt(dis) / abs(a);
    z1 = xe - dx;
    z2 = xe + dx;
    if (abs(z1) <= 1.0){
      nz += 1;
    }
    if (abs(z2) <= 1.0){
      nz += 1;
    }
    if (z1 < -1.0){
      z1 = z2;
    }
  }
  quadout[0] = nz;
  quadout[1] = z1;
  quadout[2] = z2;
  quadout[3] = xe;
  quadout[4] = ye;
}

float frac(float x) {
  x = x - floor(x);
  if (x < 0){
    x += 1;
  }
  return x;
}

void minimoon(float t) {
  float p2 = 6.283185307, arc = 206264.8062, coseps = 0.91748, sineps = 0.39778;
  float L0, L, LS, F, D, H, S, N, DL, CB, L_moon, B_moon, V, W, X, Y, Z, RHO;
  
  L0 = frac(0.606433 + 1336.855225 * t);	// mean longitude of moon
  L = p2 * frac(0.374897 + 1325.552410 * t); //mean anomaly of Moon
  LS = p2 * frac(0.993133 + 99.997361 * t); //mean anomaly of Sun
  D = p2 * frac(0.827361 + 1236.853086 * t); //difference in longitude of moon and sun
  F = p2 * frac(0.259086 + 1342.227825 * t); //mean argument of latitude

  // corrections to mean longitude in arcsec
  DL =  22640 * sin(L);
  DL += -4586 * sin(L - 2*D);
  DL += 2370 * sin(2*D);
  DL +=  769 * sin(2*L);
  DL +=  -668 * sin(LS);
  DL +=  -412 * sin(2*F);
  DL +=  -212 * sin(2*L - 2*D);
  DL +=  -206 * sin(L + LS - 2*D);
  DL +=  192 * sin(L + 2*D);
  DL +=  -165 * sin(LS - 2*D);
  DL +=  -125 * sin(D);
  DL +=  -110 * sin(L + LS);
  DL +=  148 * sin(L - LS);
  DL +=   -55 * sin(2*F - 2*D);

  // simplified form of the latitude terms
  S = F + (DL + 412 * sin(2*F) + 541 * sin(LS)) / arc;
  H = F - 2*D;
  N =   -526 * sin(H);
  N +=   44 * sin(L + H);
  N +=   -31 * sin(-L + H);
  N +=   -23 * sin(LS + H);
  N +=   11 * sin(-LS + H);
  N +=   -25 * sin(-2*L + F);
  N +=   21 * sin(-L + F);

  // ecliptic long and lat of Moon in rads
  L_moon = p2 * frac(L0 + DL / 1296000);
  B_moon = (18520.0 * sin(S) + N) /arc;

  // equatorial coord conversion - note fixed obliquity
  CB = cos(B_moon);
  X = CB * cos(L_moon);
  V = CB * sin(L_moon);
  W = sin(B_moon);
  Y = coseps * V - sineps * W;
  Z = sineps * V + coseps * W;
  RHO = sqrt(1.0 - Z*Z);
  float dec = (360.0 / p2) * atan(Z / RHO);
  float ra = (48.0 / p2) * atan(Y / (X + RHO));
  if (ra < 0 ){
    ra += 24.0;
  }
  mooneq[1] = dec;
  mooneq[2] = ra;
}

void minisun(float t) {
  float p2 = 6.283185307, coseps = 0.91748, sineps = 0.39778;
  float L, M, DL, SL, X, Y, Z, RHO, ra, dec;

  M = p2 * frac(0.993133 + 99.997361 * t);
  DL = 6893.0 * sin(M) + 72.0 * sin(2 * M);
  L = p2 * frac(0.7859453 + M / p2 + (6191.2 * t + DL)/1296000);
  SL = sin(L);
  X = cos(L);
  Y = coseps * SL;
  Z = sineps * SL;
  RHO = sqrt(1 - Z * Z);
  dec = (360.0 / p2) * atan(Z / RHO);
  ra = (48.0 / p2) * atan(Y / (X + RHO));
  if (ra < 0 ){
    ra += 24;
  }
  mooneq[1] = dec;
  mooneq[2] = ra;
}

float sin_alt(float iobj, float mjd0, int hour, float glong, float cglat, float sglat) {
  float mjd, t, ra, dec, tau, salt, rads = 0.0174532925;
  mjd = mjd0 + hour/24.0;
  t = (mjd - 51544.5) / 36525.0;
  if ((int) iobj == 1) {
    minimoon(t);
  }
  else {
    minisun(t);
  }
  ra = mooneq[2];
  dec = mooneq[1];
  tau = 15.0 * (lmst(mjd, glong) - ra);
  salt = sglat * sin(rads*dec) + cglat * cos(rads*dec) * cos(rads*tau);
  return salt;
}

float lmst(float mjd, float glong) {
  float lst, t, d;
  d = mjd - 51544.5;
  t = d / 36525.0;
  lst = range(280.46061837 + 360.98564736629 * d + 0.000387933 *t*t - t*t*t / 38710000);
  return (lst/15.0 + glong/15);
}

float range(float x) {
  float a, b;
  b = x / 360;
  a = 360 * (b - ipart(b));
  if (a  < 0 ) {
    a = a + 360;
  }
  return a;
}

float ipart(float x) {
  if (x > 0) {
    return floor(x);
  }
  else {
    return ceil(x);
  }
}

void printTime(float ftime){
  Serial.print( (int) floor(ftime) );
  Serial.print( ':' );
  Serial.println( (int) floor((ftime - floor(ftime)) * 60) );
}

void checkSunMoon(){
  float mjd = getmjd(GPS.day, GPS.month, 2000 + GPS.year, 0.0);
  
  float glat = floor(GPS.latitude / 100);
  float minutes = floor( GPS.latitude - 100 * glat );
  float seconds = GPS.latitude - 100 * glat - minutes;
  glat += minutes / 60 + seconds / 36;
  if(GPS.lat == south){
    glat *= -1;
  }
  
  float glong = floor(GPS.longitude / 100);
  minutes = floor( GPS.longitude - 100 * glong );
  seconds = GPS.longitude - 100 * glong - minutes;
  glong += minutes / 60 + seconds / 36;
  if(GPS.lon == west){
    glong *= -1;
  }
  
  //Serial.print(glat);
  //Serial.print(",");
  //Serial.println(glong);
 
  float sglong, sglat, date, ym, yz, utrise, utset, j;
  float yp, nz, hour, z1, z2, iobj;
  float rads = 0.0174532925;
  boolean rise, sett, above;

  float sinho[4];
  String always_up = " ****";
  String always_down = " ....";
  String outstring = "";

  sinho[0] = sin(rads * -0.833);
  sinho[1] = sin(rads *  -6.0);
  sinho[2] = sin(rads * -12.0);
  sinho[3] = sin(rads * -18.0);
  sglat = sin(rads * glat);
  float cglat = cos(rads * glat);
  date = mjd - tz/24;

  for (int j = 0; j < 1; j++) { // originally j < 4
    rise = false;
    sett = false;
    above = false;
    hour = 1.0;
    ym = sin_alt(2, date, hour - 1.0, glong, cglat, sglat) - sinho[j];
    if (ym > 0.0){
      above = true;
    }
    while(hour < 25 && (sett == false || rise == false)) {
      yz = sin_alt(2, date, hour, glong, cglat, sglat) - sinho[j];
      yp = sin_alt(2, date, hour + 1.0, glong, cglat, sglat) - sinho[j];
      quad(ym, yz, yp);
      nz = quadout[0];
      z1 = quadout[1];
      z2 = quadout[2];
      float xe = quadout[3];
      float ye = quadout[4];
      
      // case when one event is found in the interval
      if (((int) nz) == 1) {
        if (ym < 0.0) {
          utrise = hour + z1;
          rise = true;
        }
        else {
          utset = hour + z1;
          sett = true;
        }
      }
      if (((int) nz) == 2) {
        if (ye < 0.0) {
          utrise = hour + z2;
          utset = hour + z1;
        }
        else {
          utrise = hour + z1;
          utset = hour + z2;
        }
      }
      ym = yp;
      hour += 2.0;
    }
    if (rise == true || sett == true ) {
      if (rise == true){
        Serial.print("rise: ");
        printTime( utrise );
      }
      if (sett == true){
         Serial.print("set: ");
         printTime(utset);
      }
    }
    else {
      Serial.println("neither rise nor set");
    }
  }
  
  setTime( GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, 2000 + GPS.year);
  
  float today = now( ) * 1.0;
  moon_day( today );

  //Serial.println( outstring );
}


float GetFrac(float fr) {
  return (fr - floor(fr));
};

float getJulian(float today) {
    return ((today / 86400.0) + ( tz / 24.0 ) + 2440587.5);
};

float moon_day(float today) {
    float Ko, T, T2, T3, Jo, Fo, Mo, Mi, Bi, oldJ;
    float thisJD = getJulian( today );
    float degToRad = 3.14159265 / 180.0;
    Ko = floor((GPS.year + 100) * 12.3685);
    T = (GPS.year + 100.5) / 100.0;
    T2 = T * T;
    T3 = T * T * T;
    Jo = 2415020.0 + 29.0 * Ko;
    Fo = 0.0001178 * T2 - 0.000000155 * T3 + (0.75933 + 0.53058868 * Ko) - (0.000837 * T + 0.000335 * T2);
    Mo = 360.0 * (GetFrac(Ko * 0.08084821133)) + 359.2242 - 0.0000333 * T2 - 0.00000347 * T3;
    Mi = 360.0 * (GetFrac(Ko * 0.07171366128)) + 306.0253 + 0.0107306 * T2 + 0.00001236 * T3;
    Bi = 360.0 * (GetFrac(Ko * 0.08519585128)) + 21.2964 - (0.0016528 * T2) - (0.00000239 * T3);
    float phase = 0;
    float jday = 0;
    while (jday < thisJD) {
        float F = Fo + 1.530588 * phase;
        float M5 = (Mo + phase * 29.10535608) * degToRad;
        float M6 = (Mi + phase * 385.81691806) * degToRad;
        float B6 = (Bi + phase * 390.67050646) * degToRad;
        F -= 0.4068 * sin(M6) + (0.1734 - 0.000393 * T) * sin(M5);
        F += 0.0161 * sin(2 * M6) + 0.0104 * sin(2 * B6);
        F -= 0.0074 * sin(M5 - M6) - 0.0051 * sin(M5 + M6);
        F += 0.0021 * sin(2 * M5) + 0.0010 * sin(2 * B6 - M6);
        F += 0.5 / 1440.0;
        oldJ = jday;
        jday = Jo + 28.0 * phase + floor(F);
        phase++;
    }

    // 29.53059 days per lunar month
    float finalphase = (((thisJD - oldJ) / 29.53059));
    
    Serial.print(finalphase);
    Serial.print(" = ");
    
    if ((finalphase <= 0.0625) || (finalphase > 0.9375)) {
        Serial.println("New Moon");
    } else if (finalphase <= 0.1875) {
        Serial.println("1/4 Moon");
    } else if (finalphase <= 0.3125) {
        Serial.println("1/2 Moon");
    } else if (finalphase <= 0.4375) {
        Serial.println("3/4 Moon");
    } else if (finalphase <= 0.5625) {
        Serial.println("Full Moon");
    } else if (finalphase <= 0.6875) {
        Serial.println("3/4 Moon");
    } else if (finalphase <= 0.8125) {
        Serial.println("1/2 Moon");
    } else if (finalphase <= 0.9375) {
        Serial.println("1/4 Moon");
    }
}