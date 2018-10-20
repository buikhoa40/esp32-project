/*
   FFC Team
   FITHOU - VN
   Main sketch
*/

/* main sketch file: global include for other files */

#include <Arduino.h>

#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_deep_sleep.h>
#include <soc/rtc.h>

#include <SimpleTimer.h>
//#include <freertos/FreeRTOS.h>
//#include <freertos/semphr.h>

#include <Wire.h>
#include <WiFi.h>

#include <U8g2lib.h>

#include <driver/gpio.h>
//#include <SPI.h>

/* Constants */
#define U8G2_SDA_PIN 5
#define U8G2_SCL_PIN 4

#define BAUD 115200

#define WIFI_SSID "KHOA"
#define WIFI_PSK "aaaaaaaa"

#define NTP_SRV "asia.pool.ntp.org"

#define SCREEN_W 128
#define SCREEN_H 64

#define TOUCH_0_PIN T0 // 4: used
#define TOUCH_1_PIN T1
#define TOUCH_2_PIN T2
#define TOUCH_3_PIN T3
#define TOUCH_4_PIN T4
#define TOUCH_5_PIN T5
#define TOUCH_6_PIN T6
#define TOUCH_7_PIN T7
#define TOUCH_8_PIN T8
#define TOUCH_9_PIN T9
// Qua không khí thì touch threshold thường là 100-120, khi chạm gần là 5-15, kết nối trực tiếp là 1
#define TOUCH_THRESHOLD 16
// tối đa 1200ms chỉ nhận 1 lần touch
#define TOUCH_MAXIMUM_TIME 1200
// chạm tối thiếu 150ms để nhận touch
#define TOUCH_MINIMUM_TIME 150


/* Display config here */
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, U8G2_SCL_PIN, U8G2_SDA_PIN, U8X8_PIN_NONE);
/* Timer handler */
SimpleTimer stimer;

TaskHandle_t gpio_isr_task;
SemaphoreHandle_t gpio_isr_semaphore;

uint8_t count = 0;
uint8_t activity = 0;
int timer[4] = { -1, -1, -1, -1};

//#include "esp32_funcs.cpp"
//#include "esp32_clock.cpp"
//#include "esp32_ble.cpp"
//#include "esp32_http_srv.cpp"
//#include "esp32_mpu.cpp"

struct configData {
  uint8_t brightness = 100;
  char pin[5] = "4321";
  char wifi_ssid[65] = WIFI_SSID;
  char wifi_psk[65] = WIFI_PSK;
  bool ip_required = false;
  IPAddress mpu_udp_ip = IPAddress(127, 0, 0, 1);
  uint16_t mpu_udp_port = 8080;
};

struct configData config;

/* GPIO interrupt handling function */
/*
  static void IRAM_ATTR gpioISRSend (void* arg) {
  xSemaphoreGiveFromISR(gpio_isr_semaphore, NULL);
  }
*/

/* GPIO interrupt handling task */
/*
  static void gpioISRTask (void* arg) {
  while (true) {
    if (xSemaphoreTake(gpio_isr_semaphore, portMAX_DELAY) == pdTRUE) {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      if (timer[0] == -1) {
        startScreen();
        timer[0] = stimer.setInterval(1000, intervalDisplayTime);
      } else {
        stimer.deleteTimer(timer[0]);
        timer[0] = -1;
        clearScreen();
        //startSleepState();
      }
    }
  }
  }
*/

/*
  void intervalReadTouch(void) {
  Serial.println(touchRead(TOUCH_4_PIN));
  }
*/

unsigned long button_hdl_temp;
unsigned long button_hdl_presstime = 0l;
uint8_t button_hdl_press = 0;
bool button_hdl_pressed = false;

bool button_begin_proc (int num) {
  if (button_hdl_press != num) {
    button_hdl_presstime = millis();
    button_hdl_pressed = false;
    button_hdl_press = num;
  } else if (button_hdl_pressed = true) {
    button_hdl_presstime += TOUCH_MAXIMUM_TIME - 50;
    button_hdl_pressed = false;
  }
  if (button_hdl_pressed) {
    return false;
  }
  if ((button_hdl_temp = (millis() - button_hdl_presstime)) < TOUCH_MINIMUM_TIME) {
    return false;
  }
  if (button_hdl_temp > TOUCH_MAXIMUM_TIME) {
    button_hdl_pressed = true;
    return false;
  }
  button_hdl_pressed = true;
  button_hdl_press = 0;
  return true;
}

void button_hdl_3 (void) {
  if (button_begin_proc(3)) {
    Serial.printf("button %d pressed\n", 3);
    Serial.println("toggle display brightness");
    if (config.brightness == 255) {
      config.brightness = 100;
    } else {
      config.brightness = 255;    
    }
    u8g2.setContrast(config.brightness);
  }
}

void button_hdl_4 (void) {
  if (button_begin_proc(4)) {
    Serial.printf("button %d pressed\n", 4);
  }
}

void button_hdl_5 (void) {
  if (button_begin_proc(5)) {
    Serial.printf("button %d pressed\n", 5);  
  }
}

void button_hdl_6 (void) {
  if (button_begin_proc(6)) {
    Serial.printf("button %d pressed\n", 6);
  }
}

void button_hdl_7 (void) {
  if (button_begin_proc(7)) {
    Serial.printf("button %d pressed\n", 7);
  }
}

/* setup wifi */
void setup_wifi(bool ip_required) {
  Serial.printf("WiFi: Connecting to %s", config.wifi_ssid);
  //WiFi.begin(config.wifi_psk, config.wifi_psk);

  WiFi.begin(WIFI_SSID, WIFI_PSK);

  if (!ip_required) {
    Serial.println("...");
    return;
  }

  int i = 0;
wifiLoop:

  while (WiFi.status() != WL_CONNECTED) {
    i++;
    if ((i % 240) == 0) {
      Serial.println("Connected seem failed, select another AP:");
      Serial.print("SSID: ");
      String str = "";
      uint8_t t;
      while (Serial.available()) {
        while (Serial.available() > 0 && (t == (uint8_t)Serial.read()) != '\n') {
          str += t;
        }
      }
      str.toCharArray(config.wifi_ssid, sizeof(config.wifi_ssid));
      Serial.print("PSK: ");
      str = "";
      while (Serial.available()) {
        while (Serial.available() > 0 && (t == (uint8_t)Serial.read()) != '\n') {
          str += t;
        }
      }
      str.toCharArray(config.wifi_psk, sizeof(config.wifi_psk));
      goto wifiLoop;
    }
    delay(100);
    Serial.print(".");
  }
  Serial.println(" OK");
  Serial.print("WiFi: IP = ");
  /* not like it */
  Serial.println(WiFi.localIP());
}

/* boot up function */
void setup(void) {
  /* baud rate in Serial Monitor must match with this */
  Serial.begin(BAUD);

  while (!Serial) {
    delay(4);
  }

  Serial.printf("ESP32 Serial = %d OK\n", BAUD);
  Serial.print("ESP32 SDK: ");
  Serial.println(ESP.getSdkVersion());
  uint64_t chipid = ESP.getEfuseMac();
  Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(chipid >> 32));
  Serial.printf("%08X\n", (uint32_t)chipid);
  Serial.printf("CPU core %01d is main app0 processor\n", xPortGetCoreID());
  //Serial.printf("CPU freq is changing\n");
  //rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);
  Serial.printf("CPU freq = %d mhz OK\n", 240);

  delay(100);

  //u8g2.initDisplay();
  //u8g2.setPowerSave(0);
  u8g2.begin();
  u8g2.setContrast(config.brightness);
  Serial.println("Display initialization OK");

  u8g2.firstPage();
  do {
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_logisoso26_tn);
    u8g2.drawStr(0, 64, "BOOT");
  } while (u8g2.nextPage());

  delay(100);

  //setup_ble();

  delay(100);

  setup_wifi(config.ip_required);

  delay(100);

  setup_clock();

  //WiFi.disconnect(true);
  //WiFi.mode(WIFI_OFF);

  delay(100);

  setup_mpu();

  delay(100);

  Serial.println("Starting activity = 0 (DisplayTime)");
  timer[0] = stimer.setInterval(1000, intervalDisplayTime);

  //Serial.println("Starting activity = 1 (ReadTouch)");
  //timer[1] = stimer.setInterval(1000, intervalReadTouch);
  //Serial.println("Starting activity = 0 (PrintBluetooth)");
  //timer[1] = stimer.setInterval(1000, intervalPrintBluetooth);
  //Serial.println("Starting activity = 0 ()");
  //timer[2] = stimer.setInterval(1000, interval);

  delay(100);

  setup_http_srv();

  delay(100);

  Serial.print("Setup buttons using advanced GPIO interrupt handler...");

  touchAttachInterrupt(TOUCH_3_PIN, button_hdl_3, TOUCH_THRESHOLD);
  touchAttachInterrupt(TOUCH_4_PIN, button_hdl_4, TOUCH_THRESHOLD);
  touchAttachInterrupt(TOUCH_5_PIN, button_hdl_5, TOUCH_THRESHOLD);
  touchAttachInterrupt(TOUCH_6_PIN, button_hdl_6, TOUCH_THRESHOLD);
  touchAttachInterrupt(TOUCH_7_PIN, button_hdl_7, TOUCH_THRESHOLD);

  /*
    gpio_isr_semaphore = xSemaphoreCreateBinary();
    gpio_pad_select_gpio(0);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO_NUM_0, GPIO_INTR_NEGEDGE);
  */
  /* Offload this task to RF core */
  /*
    xTaskCreatePinnedToCore(
      gpioISRTask,            // pvTaskCode
      "gpioisrtask1",         // pcName
      1024,                   // usStackDepth
      NULL,                   // pvParameters
      10,                     // uxPriority
      &gpio_isr_task,         // pxCreatedTask
      1);                     // xCoreID - 0 RF core
  */
  /*
    xTaskCreate(gpioISRTask, "gpioisrtask1", 1024, NULL, 10, NULL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_0, gpioISRSend, (void*)GPIO_NUM_0);
  */
  Serial.println(" OK");

  delay(100);

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Setup COMPLETED, LED on.");

}

/* runtime loop function */
void loop (void) {

  /* main timer handler */

  stimer.run();

  if (WiFi.status() == WL_CONNECTED) {
    loop_http_srv();
  }

  loop_mpu();

  /*
    if (hallRead() > 10) {
      Serial.println("Hall sensor triggered: magnet closed");
    }
  */

  /* Let it run at 250hz */
  delay(4);
}


