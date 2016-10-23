#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "mma8653.h"
//#include <ESP8266WiFi.h>
//#include <WiFiUdp.h>


#define LED_ROWS 8
#define LED_COLS 8
#define LED_COUNT (LED_ROWS*LED_COLS)
#define BUTTON 4

const int i2c_scl = 14;
const int i2c_sda = 12;


class Pixel {
  /** Blend the new value in with the fraction specified 0-256*/
public:
  uint8_t v;
  void blend(int _v, int frac) {
    *this = (v*(256-frac)+(_v*frac))/256;
  };
  uint8_t operator=(int in) {
    v = in;
  };

  void draw() {
    int v2 = v;
    if (v2 == 255) v2=254;    
    Serial1.print(char(v2));
  }
};

class Color {
  Pixel r;
  Pixel g;
  Pixel b;
  
public:
  void blend(int r, int g, int b, int frac) {
    if (frac < 0) frac = 0;
    if (frac > 255) frac = 255;
    this->r.blend(r,frac);
    this->b.blend(b,frac);
    this->g.blend(g,frac);
  }
  void set(int r, int g, int b) {
    this->r = r;
    this->g = g;
    this->b = b;
  }
  void draw() {
    r.draw();
    g.draw();
    b.draw();
  }

};
  
class Board {
  Color *color = new Color[LED_COUNT]();
  boolean isChanged = false;
  uint32_t last_draw = 0;
public:
  uint8_t brightness = 0;
  void setup() {
    // Drawing in matrix.
    Serial1.begin( 230400 );
  }
  
  Color *cell(int x, int y) {
    if (x < 0 || x > 7) return color;
    if (y < 0 || y > 7) return color;
    return color+8*x+y;
  }

  void clear(int r, int g, int b) {
    isChanged = true;
    for (int i = 0 ; i < LED_COUNT ; i++) {
      (color+i)->set(r,g,b);    
    }
  }

  void set(int x, int y, int r, int g, int b) {
    isChanged = true;
    cell(x,y)->set(r,g,b);
  }

  void blend(int x, int y, int r, int g, int b, int blend) {
    isChanged = true;
    cell(x,y)->blend(r,g,b,blend);
  }

  void setBits(int row, uint8_t val, int r, int g, int b) {
    isChanged = true;
    for (int ii = 0 ; ii < 8 ; ii++) {
      int mask = 1 << ii;
      if ((val & mask) != 0)
	cell(row,ii)->set(r,g,b);
    }
  }


  void draw(boolean force) {
    const uint32_t now = millis();
    if (!force && (!isChanged || now-last_draw < 40))
      return;
    last_draw = now;
    isChanged = false;

    // Make first pixel oscillate.
    //Serial1.print((char(now>>3) & 0xfe));
    //Serial1.print((char(now>>3) & 0xfe));
    //Serial1.print((char(now>>3) & 0xfe));

    for (int i = 0 ; i < LED_COUNT ; i++) {
      Color *j= color+i;
      j->draw();

    }
    Serial1.print(char(255));

    // Command mode.
    for(int i = 0; i < 9; i++) {
      Serial1.print(char(255));
    }
    Serial1.print(char(1)); // 'Set brightness'
    Serial1.print(char(brightness)); 
  }

  /** Valid from 0 -- 255 */
  void setBrightness(int val) {
    brightness = val;
    isChanged = true;
  } 
};

class TheButton {
public:
  boolean nowDown = false;
  // If this is true, you can get durationDown.
  boolean justCameUp = false;
  uint32_t whenDown;
  uint32_t durationDown;
  void setup () {
    pinMode(BUTTON, INPUT_PULLUP);
  }

  void loop() {
    boolean nowDown2 = isNowDown2();
    if (nowDown2 && !nowDown) {
      // Just going down the first time.
      whenDown = millis();
    }
    justCameUp = nowDown && !nowDown2;
    if (justCameUp) {
      durationDown = millis()-whenDown;
    }
    nowDown = nowDown2;
  }

private:
  boolean isNowDown2() {
    return !digitalRead(BUTTON);
  }

  
};

#define ACCEL_EMWA (1<<1)

class AccelRaw {
  float x, y, z;
  void update(x,y,z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }
}
  class AccelEMWA {
    const float ACCEL_EMWA = (1<<4);
    float x, y, z;
    void update(x,y,z) {
      emwa(this->x,x);
      emwa(this->y,y);
      emwa(this->z,z);
    }
    float emwa(float &acc, float in) {
      acc = (acc*(ACCEL_EMWA-1)+in)/ACCEL_EMWA;
    }
  };

class Accel {
  MMA8653 chip;
public:
  // Some notes:
  // 1 g is about 16 as measured by this.
  // az is negative 16.
    
  // Accelerometer values at last read. Note, 45 degree orientation: '-x' is with twosigma down and on right.
  float ax, ay, az;
  // EMWA values of above. 
  float ex, ey, ez;
  // EMWA, but displaced to the speed in x and y.
  // dz is about -16 with 'two sigma' text on bottom.
  // 'x' measures rows. 'y' measures columns.
  float dx, dy;
  // Shock accel value (detect bump)
  float accel;



  void setup() {
    // Accelerometer setup.
    Wire.begin(i2c_sda, i2c_scl);
    chip.setup();
  }

  // Invoke each iteration.
  void loop() {
    chip.getXYZ(ax,ay,az);
    ex=emwa(ex,ax);
    ey=emwa(ey,ay);
    ez=emwa(ez,az);
    accel = sqrt(ex*ex+ey*ey+ez*ez);

    // From the diagram, sensor assumes 45 degree mounting. Rotate it.
    const float mag = sqrt(ex*ex+ey*ey);
    const float angle = atan2(ex, -ey);
    dx = -sin(angle + 45 * M_PI/180) * mag;
    dy = cos(angle + 45 * M_PI/180) * mag;
  }
  
};


Board board = Board();

TheButton button = TheButton();

Accel accel = Accel();


class Demo {
public:
  virtual void setup() = 0;
  virtual void loop() = 0;
};

// Show clock, accel, and switch parameters.
class OrientationDemo : public Demo {
public:
  virtual void setup() {};
  virtual void loop() {
    const uint32_t now = millis();

    board.clear(0,0,16);
    board.setBits(0,now >> 9,0,0,254); 
    //board.set(5,5,255,0,0);
    //board.set(6,6,0,255,0);
    //board.set(7,7,0,0,255);

    showAccelNicely(2,8*accel.ex);
    showAccelNicely(3,8*accel.ey);
    showAccelNicely(4,8*accel.ez);
    board.setBits(5,board.brightness,255,255,255);
    showAccelNicely(6,8*accel.dx);
    showAccelNicely(7,8*accel.dy);

    if (button.nowDown) 
      board.set(1,0,255,128,128);
    else
      board.set(1,0,128,255,128);
    if (button.justCameUp)
      board.set(1,1,255,128,128);
    else
      board.set(1,1,128,255,128);

    board.setBits(1,(button.durationDown>>6)<<2,255,255,255); 

    
  }  


  void showAccelNicely(int col, float accel) {
    if (accel > 0)
      board.setBits(col,accel,0,255,0);
    else
      board.setBits(col,-accel,255,0,0);
  }
};

// Sliders track 16 bits internally, even though they only output/store 8 bits. That lets them handle variable accelerations.
class Slider {
  uint16_t minVal;
  uint16_t maxVal;
  boolean wrap;
  int currentVal;
public:
  // Should we wrap when we get to max/min?
  Slider(uint8_t minVal, uint8_t maxVal, uint8_t currentVal, boolean wrap) {
    this->minVal = minVal << 8;
    this->maxVal = (maxVal << 8) + 255;
    this->currentVal = currentVal<<8 + 128;
    this->wrap = wrap;
  }
  // delta is a fixedpoint with 8 fractional bits.
  void accountForDelta(int delta) {
    int maxChange = maxVal-minVal;
    if (delta > maxChange) delta = maxChange;
    if (delta < -maxChange) delta = -maxChange;
    currentVal = currentVal + delta;
    if (!wrap) {
      if (currentVal < minVal) currentVal = minVal;
      if (currentVal > maxVal) currentVal = maxVal;
    } else {
      if (currentVal < minVal) currentVal += maxChange;
      if (currentVal > maxVal) currentVal -= maxChange;
    }
  }
  uint8_t value() {
    return currentVal>>8;
  }
  
};


class PickBrightness : public Demo {
  Slider brightness = Slider(1,255,board.brightness,false);
public:
  virtual void setup() {};
  virtual void loop() {
    //board.setBits(0,board.brightness,255,255,255);
    brightness.accountForDelta(accel.dy*64);
    board.setBrightness(brightness.value());
  }

  
};



OrientationDemo orientationDemo = OrientationDemo();
PickBrightness pickBrightness = PickBrightness();
void setup() {
  // put your setup code here, to run once:
  //WiFi.persistent(false);
  //WiFi.mode(WIFI_STA);
  //WiFi.begin("netname", "passwd");

  // Communication to PC
  Serial.begin ( 460800 );

  board.setup();
  button.setup();
  accel.setup();
  board.setBrightness(64);
}

void loop() {
  const uint32_t now = millis();
  accel.loop();
  button.loop();
  orientationDemo.loop();
  pickBrightness.loop();
  board.draw(false);
  delay(20);
}
/*
  void loopTEMP() {
  
  const uint32_t now = millis();

  // only draw the LEDs at 30Hz
  if (!do_draw && now - last_draw_millis < 30)
  return;
  last_draw_millis = now;
  
  if (draw_video)
  pixels.draw(badge.matrix);
  else
  demo->draw(badge.matrix);

  badge.matrix.show();
  }
*/
