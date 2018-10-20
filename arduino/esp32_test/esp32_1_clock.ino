/*
   FFC Team
   FITHOU - VN
   Clock functions
*/

#define RTC_DS_PIN 2
#define RTC_SDA_PIN 16
#define RTC_SCL_PIN 17

#define GMT_OFFSET 3600*7
#define GMT_DST 0

#include <time.h>

// directly from esp32-hal-time.c
void setTimeZone(long offset, int daylight) {
    char cst[16] = {0};
    char cdt[16] = "DST";
    char tz[32] = {0};

    if(offset % 3600){
        sprintf(cst, "UTC%ld:%02u:%02u", offset / 3600, abs((offset % 3600) / 60), abs(offset % 60));
    } else {
        sprintf(cst, "UTC%ld", offset / 3600);
    }
    if(daylight != 3600){
        long tz_dst = offset - daylight;
        if(tz_dst % 3600){
            sprintf(cdt, "DST%ld:%02u:%02u", tz_dst / 3600, abs((tz_dst % 3600) / 60), abs(tz_dst % 60));
        } else {
            sprintf(cdt, "DST%ld", tz_dst / 3600);
        }
    }
    sprintf(tz, "%s%s", cst, cdt);
    setenv("TZ", tz, 1);
    tzset();
}

/* RTC device - type DS1307 (I2C) */
RTC_DS1307 rtc;

/* lấy tên ngày */
char* getWeekDayStr (int wday) {
  switch (wday) {
    case 0:
      return "Mon";
      break;
    case 1:
      return "Tue";
      break;
    case 2:
      return "Wed";
      break;
    case 3:
      return "Thu";
      break;
    case 4:
      return "Fri";
      break;
    case 5:
      return "Sta";
      break;
    case 6:
      return "Sun";
      break;
  }
}

/* timer function - run once per second - display time to U8g2 display */
void intervalDisplayTime (void) {
  static struct tm timeinfo;
  char timestr[2 + 1 + 2 + 1 + 2 + 1];
  static int old_yday = 0;
  static char old_datestr[3 + 2 + 2 + 1 + 2 + 1 + 4 + 1];
  //Serial.printf("%d\n", count);
  count++;
  u8g2.firstPage();
  if (!getLocalTime(&timeinfo)) {
    Serial.println("getLocalTime failed");
    return;
  }
  do {
    if (old_yday != timeinfo.tm_yday || old_yday == 0) {
      old_yday = timeinfo.tm_yday;
      snprintf(old_datestr, sizeof(old_datestr), "%s, %02d-%02d-%04d", getWeekDayStr(timeinfo.tm_wday), timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
    }
    u8g2.setDrawColor(2);
    u8g2.setFont(u8g2_font_7x14B_mr);
    u8g2.drawStr(10, 20, old_datestr);
    snprintf(timestr, sizeof(timestr), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_logisoso26_tn);
    u8g2.drawStr(0, 64, timestr);
  } while (u8g2.nextPage());
}

/* runtime loop function */
void setup_clock() {
  bool rtc_found = false;
  Serial.print("DS1307 Status... ");
  if (!rtc.begin(RTC_SDA_PIN, RTC_SCL_PIN, 100000)) {
    Serial.println("NOT found");
  } else {
    rtc_found = true;
    Serial.println("OK");
  }

  if (!rtc_found || !rtc.isrunning()) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Setting time with NTP srv %s", NTP_SRV);
      configTime(GMT_OFFSET, GMT_DST, NTP_SRV);
      Serial.print(".");
      struct tm timeinfo;
      while (!getLocalTime(&timeinfo)) {
        delay(100);
        Serial.print(".");
      }
      Serial.println(" OK");
      if (rtc_found) {
        rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
        Serial.println("DS1307 now initialized.");
      }
    } else {
      setTimeZone(GMT_OFFSET, GMT_DST);
      if (rtc_found) {
        rtc.adjust(DateTime(2010, 1, 1, 0, 0, 0));
        Serial.println("WiFi not connected, NTP first time syncing not available, defaults value applied.");
      } else {
        Serial.println("WiFi not connected, no time check available.");
      }
    }
  } else {
    setTimeZone(GMT_OFFSET, GMT_DST);
    /* DateTime to struct timeval */
    uint32_t now1 = (uint32_t)rtc.now().unixtime();
    struct timeval now0 = { .tv_sec = now1 };
    settimeofday(&now0, NULL);
    Serial.print("DS1307 time read = ");
    Serial.print(now0.tv_sec);
    Serial.print(" (");
    Serial.print(now1);
    Serial.println(")");
  }

  delay(100);
}


