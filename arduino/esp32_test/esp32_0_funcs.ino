/*
 * FFC Team
 * FITHOU - VN
 * Misc functions
*/

/* bật màn hình lại sau clearScreen() */
void startScreen(void) {
  u8g2.setPowerSave(0);
  u8g2.setContrast(config.brightness);
}

/* tắt màn hình */
void clearScreen (void) {
  u8g2.firstPage();
  do {
    u8g2.drawBox(0, 0, SCREEN_W, SCREEN_H);
  } while (u8g2.nextPage());
  u8g2.clear();
  u8g2.setContrast(0);
  u8g2.setPowerSave(1);
}

/* đi vào chế độ ngủ */
void startSleepState(void) {
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
  // gpio_pullup_en(GPIO_INPUT_IO_TRIGGER);
  // gpio_pulldown_dis(GPIO_INPUT_IO_TRIGGER);
  // esp_deep_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER, LOW);
  Serial.println("deep sleep");
  esp_deep_sleep_start();
}

// RTClib.cpp

// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!
// Modified for simatimulous I2C

#include <pgmspace.h>

#define DS1307_ADDRESS  0x68
#define DS1307_CONTROL  0x07
#define DS1307_NVRAM    0x08

#define SECONDS_PER_DAY 86400L

#define SECONDS_FROM_1970_TO_2000 946684800

// Timespan which can represent changes in time with seconds accuracy.
class TimeSpan {
public:
    TimeSpan (int32_t seconds = 0);
    TimeSpan (int16_t days, int8_t hours, int8_t minutes, int8_t seconds);
    TimeSpan (const TimeSpan& copy);
    int16_t days() const         { return _seconds / 86400L; }
    int8_t  hours() const        { return _seconds / 3600 % 24; }
    int8_t  minutes() const      { return _seconds / 60 % 60; }
    int8_t  seconds() const      { return _seconds % 60; }
    int32_t totalseconds() const { return _seconds; }

    TimeSpan operator+(const TimeSpan& right);
    TimeSpan operator-(const TimeSpan& right);

protected:
    int32_t _seconds;
};

class DateTime {
public:
    DateTime (uint32_t t =0);
    DateTime (uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour =0, uint8_t min =0, uint8_t sec =0);
    DateTime (const DateTime& copy);
    DateTime (const char* date, const char* time);
    DateTime (const __FlashStringHelper* date, const __FlashStringHelper* time);
    uint16_t year() const       { return 2000 + yOff; }
    uint8_t month() const       { return m; }
    uint8_t day() const         { return d; }
    uint8_t hour() const        { return hh; }
    uint8_t minute() const      { return mm; }
    uint8_t second() const      { return ss; }
    uint8_t dayOfTheWeek() const;

    // 32-bit times as seconds since 1/1/2000
    long secondstime() const;   
    // 32-bit times as seconds since 1/1/1970
    uint32_t unixtime(void) const;

    DateTime operator+(const TimeSpan& span);
    DateTime operator-(const TimeSpan& span);
    TimeSpan operator-(const DateTime& right);

protected:
    uint8_t yOff, m, d, hh, mm, ss;
};

// RTC based on the DS1307 chip connected via I2C and the WirE library
enum Ds1307SqwPinMode { OFF = 0x00, ON = 0x80, SquareWave1HZ = 0x10, SquareWave4kHz = 0x11, SquareWave8kHz = 0x12, SquareWave32kHz = 0x13 };

class RTC_DS1307 {
public:
    boolean begin(void);
    boolean begin(int sda, int scl, int clock);
    static void adjust(const DateTime& dt);
    uint8_t isrunning(void);
    static DateTime now();
    static Ds1307SqwPinMode readSqwPinMode();
    static void writeSqwPinMode(Ds1307SqwPinMode mode);
    uint8_t readnvram(uint8_t address);
    void readnvram(uint8_t* buf, uint8_t size, uint8_t address);
    void writenvram(uint8_t address, uint8_t data);
    void writenvram(uint8_t address, uint8_t* buf, uint8_t size);
};

// RTC using the internal millis() clock, has to be initialized before use
// NOTE: this clock won't be correct once the millis() timer rolls over (>49d?)
class RTC_Millis {
public:
    static void begin(const DateTime& dt) { adjust(dt); }
    static void adjust(const DateTime& dt);
    static DateTime now();

protected:
    static long offset;
};

//

static uint8_t read_i2c_register(uint8_t addr, uint8_t reg) {
  Wire1.beginTransmission(addr);
  Wire1.write((byte)reg);
  Wire1.endTransmission();
  Wire1.requestFrom(addr, (byte)1);
  return Wire1.read();
}

static void write_i2c_register(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(addr);
  Wire1.write((byte)reg);
  Wire1.write((byte)val);
  Wire1.endTransmission();
}

const uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };

static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

DateTime::DateTime (uint32_t t) {
  t -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970

    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;
    for (yOff = 0; ; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap)
            break;
        days -= 365 + leap;
    }
    for (m = 1; ; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
        if (leap && m == 2)
            ++daysPerMonth;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    d = days + 1;
}

DateTime::DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000)
        year -= 2000;
    yOff = year;
    m = month;
    d = day;
    hh = hour;
    mm = min;
    ss = sec;
}

DateTime::DateTime (const DateTime& copy):
  yOff(copy.yOff),
  m(copy.m),
  d(copy.d),
  hh(copy.hh),
  mm(copy.mm),
  ss(copy.ss)
{}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

// A convenient constructor for using "the compiler's time":
//   DateTime now (__DATE__, __TIME__);
// NOTE: using F() would further reduce the RAM footprint, see below.
DateTime::DateTime (const char* date, const char* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (date[0]) {
        case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = date[2] == 'r' ? 4 : 8; break;
        case 'M': m = date[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(date + 4);
    hh = conv2d(time);
    mm = conv2d(time + 3);
    ss = conv2d(time + 6);
}

// A convenient constructor for using "the compiler's time":
// This version will save RAM by using PROGMEM to store it by using the F macro.
//   DateTime now (F(__DATE__), F(__TIME__));
DateTime::DateTime (const __FlashStringHelper* date, const __FlashStringHelper* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    char buff[11];
    memcpy_P(buff, date, 11);
    yOff = conv2d(buff + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (buff[0]) {
        case 'J': m = buff[1] == 'a' ? 1 : m = buff[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = buff[2] == 'r' ? 4 : 8; break;
        case 'M': m = buff[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(buff + 4);
    memcpy_P(buff, time, 8);
    hh = conv2d(buff);
    mm = conv2d(buff + 3);
    ss = conv2d(buff + 6);
}

uint8_t DateTime::dayOfTheWeek() const {    
    uint16_t day = date2days(yOff, m, d);
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

uint32_t DateTime::unixtime(void) const {
  uint32_t t;
  uint16_t days = date2days(yOff, m, d);
  t = time2long(days, hh, mm, ss);
  t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000

  return t;
}

long DateTime::secondstime(void) const {
  long t;
  uint16_t days = date2days(yOff, m, d);
  t = time2long(days, hh, mm, ss);
  return t;
}

DateTime DateTime::operator+(const TimeSpan& span) {
  return DateTime(unixtime()+span.totalseconds());
}

DateTime DateTime::operator-(const TimeSpan& span) {
  return DateTime(unixtime()-span.totalseconds());
}

TimeSpan DateTime::operator-(const DateTime& right) {
  return TimeSpan(unixtime()-right.unixtime());
}

// TimeSpan implementation

TimeSpan::TimeSpan (int32_t seconds):
  _seconds(seconds)
{}

TimeSpan::TimeSpan (int16_t days, int8_t hours, int8_t minutes, int8_t seconds):
  _seconds((int32_t)days*86400L + (int32_t)hours*3600 + (int32_t)minutes*60 + seconds)
{}

TimeSpan::TimeSpan (const TimeSpan& copy):
  _seconds(copy._seconds)
{}

TimeSpan TimeSpan::operator+(const TimeSpan& right) {
  return TimeSpan(_seconds+right._seconds);
}

TimeSpan TimeSpan::operator-(const TimeSpan& right) {
  return TimeSpan(_seconds-right._seconds);
}

////////////////////////////////////////////////////////////////////////////////
// RTC_DS1307 implementation

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }


boolean RTC_DS1307::begin(void) {
  Wire1.begin();
  Wire1.beginTransmission(DS1307_ADDRESS);
  Wire1.write((byte)0);
  if ( Wire1.endTransmission() == 0 ) {
    return true;
  }
  return false;
}

boolean RTC_DS1307::begin(int sda, int scl, int clock) {
  Wire1.begin(sda, scl, clock);
  Wire1.beginTransmission(DS1307_ADDRESS);
  Wire1.write((byte)0);
  if ( Wire1.endTransmission() == 0 ) {
    return true;
  }
  return false;
}

uint8_t RTC_DS1307::isrunning(void) {
  Wire1.beginTransmission(DS1307_ADDRESS);
  Wire1.write((byte)0);
  Wire1.endTransmission();

  Wire1.requestFrom(DS1307_ADDRESS, 1);
  uint8_t ss = Wire1.read();
  return !(ss>>7);
}

void RTC_DS1307::adjust(const DateTime& dt) {
  Wire1.beginTransmission(DS1307_ADDRESS);
  Wire1.write((byte)0); // start at location 0
  Wire1.write(bin2bcd(dt.second()));
  Wire1.write(bin2bcd(dt.minute()));
  Wire1.write(bin2bcd(dt.hour()));
  Wire1.write(bin2bcd(0));
  Wire1.write(bin2bcd(dt.day()));
  Wire1.write(bin2bcd(dt.month()));
  Wire1.write(bin2bcd(dt.year() - 2000));
  Wire1.endTransmission();
}

DateTime RTC_DS1307::now() {
  Wire1.beginTransmission(DS1307_ADDRESS);
  Wire1.write((byte)0);
  Wire1.endTransmission();

  Wire1.requestFrom(DS1307_ADDRESS, 7);
  uint8_t ss = bcd2bin(Wire1.read() & 0x7F);
  uint8_t mm = bcd2bin(Wire1.read());
  uint8_t hh = bcd2bin(Wire1.read());
  Wire1.read();
  uint8_t d = bcd2bin(Wire1.read());
  uint8_t m = bcd2bin(Wire1.read());
  uint16_t y = bcd2bin(Wire1.read()) + 2000;
  
  return DateTime (y, m, d, hh, mm, ss);
}

Ds1307SqwPinMode RTC_DS1307::readSqwPinMode() {
  int mode;

  Wire1.beginTransmission(DS1307_ADDRESS);
  Wire1.write(DS1307_CONTROL);
  Wire1.endTransmission();
  
  Wire1.requestFrom((uint8_t)DS1307_ADDRESS, (uint8_t)1);
  mode = Wire1.read();

  mode &= 0x93;
  return static_cast<Ds1307SqwPinMode>(mode);
}

void RTC_DS1307::writeSqwPinMode(Ds1307SqwPinMode mode) {
  Wire1.beginTransmission(DS1307_ADDRESS);
  Wire1.write(DS1307_CONTROL);
  Wire1.write(mode);
  Wire1.endTransmission();
}

void RTC_DS1307::readnvram(uint8_t* buf, uint8_t size, uint8_t address) {
  int addrByte = DS1307_NVRAM + address;
  Wire1.beginTransmission(DS1307_ADDRESS);
  Wire1.write(addrByte);
  Wire1.endTransmission();
  
  Wire1.requestFrom((uint8_t) DS1307_ADDRESS, size);
  for (uint8_t pos = 0; pos < size; ++pos) {
    buf[pos] = Wire1.read();
  }
}

void RTC_DS1307::writenvram(uint8_t address, uint8_t* buf, uint8_t size) {
  int addrByte = DS1307_NVRAM + address;
  Wire1.beginTransmission(DS1307_ADDRESS);
  Wire1.write(addrByte);
  for (uint8_t pos = 0; pos < size; ++pos) {
    Wire1.write(buf[pos]);
  }
  Wire1.endTransmission();
}

uint8_t RTC_DS1307::readnvram(uint8_t address) {
  uint8_t data;
  readnvram(&data, 1, address);
  return data;
}

void RTC_DS1307::writenvram(uint8_t address, uint8_t data) {
  writenvram(address, &data, 1);
}

// RTC_Millis implementation

long RTC_Millis::offset = 0;

void RTC_Millis::adjust(const DateTime& dt) {
    offset = dt.unixtime() - millis() / 1000;
}

DateTime RTC_Millis::now() {
  return (uint32_t)(offset + millis() / 1000);
}



