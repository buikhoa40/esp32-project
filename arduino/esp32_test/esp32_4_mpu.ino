/*
   FFC Team
   FITHOU - VN
   MPU6050 functions
*/

/* 
 * MPU6050 motion/accelerator sensor
 * 6DOF tracking
 * Send to external OSCServer
 * Pinout
 */

#define OUTPUT_TEAPOT_OSC

#include <driver/i2c.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>

#ifdef OUTPUT_TEAPOT_OSC
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <DNSServer.h>
#include <OSCMessage.h>
#endif

// need right configuration
#define MPU_INT_PIN 36
#define MPU_SDA_PIN 21
#define MPU_SCL_PIN 22
#define MPU_I2C_CLOCK 200000

//TwoWire Wire2(1);

MPU6050 mpu(MPU6050_ADDRESS_AD0_HIGH);

i2c_config_t i2cconf;
bool mpu_ready = false;
volatile bool mpu_interrupt = false;
uint8_t mpu_int_status;
uint16_t mpu_packet_size;
uint16_t mpu_fifo_count;
uint8_t mpu_fifo_buf[64];

void mpu_dmp_data_ready () {
  mpu_interrupt = true;
}

void setup_mpu() {
  
  // need driver i2c, clock mode 400kHz
  //Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 400000);
  //Wire2.setClock(400000);

  i2cconf.mode = I2C_MODE_MASTER;
  i2cconf.sda_io_num = (gpio_num_t)MPU_SDA_PIN;
  i2cconf.scl_io_num = (gpio_num_t)MPU_SCL_PIN;
  i2cconf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2cconf.scl_pullup_en = GPIO_PULLUP_ENABLE;

  i2cconf.master.clk_speed = MPU_I2C_CLOCK;

  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2cconf));
  
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

  Serial.print("MPU6050 initializing...");
  mpu.initialize();
  pinMode(MPU_INT_PIN, INPUT_PULLUP);

  if (mpu.testConnection()) {
    Serial.println(" OK");
    Serial.print("MPU6050 DMP6 initializing... ");
    if (mpu.dmpInitialize() == 0) {

      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788);
      mpu.setDMPEnabled(true);

      digitalPinToInterrupt(MPU_INT_PIN);
      attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpu_dmp_data_ready, RISING);

      mpu_int_status = mpu.getIntStatus();
      mpu_packet_size = mpu.dmpGetFIFOPacketSize();

      mpu_ready = true;

      Serial.println(" OK");
    } else {
      Serial.println(" failed");
    }
  } else {
    Serial.println(" failed");
  }

  // 10.42.0.1
  // 192.168.10.10
  config.mpu_udp_ip = IPAddress(10, 42, 0, 1);
  config.mpu_udp_port = 8080;

}

void loop_mpu() {

  if (!mpu_ready) {
    return;
  }

  while (!mpu_interrupt && mpu_fifo_count < mpu_packet_size) {
    if (mpu_interrupt && mpu_fifo_count < mpu_packet_size) {
      // try to get out of the infinite loop
      mpu_fifo_count = mpu.getFIFOCount();
    }
    // other program behavior stuff here
    // if you are really paranoid you can frequently test in between other stuff to see if mpu_interrupt is true, and if so, "break;" from the while() loop to immediately process the MPU data
  }

  // reset interrupt flag and get INT_STATUS byte
  mpu_interrupt = false;
  mpu_int_status = mpu.getIntStatus();

  // get current FIFO count
  mpu_fifo_count = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpu_int_status & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || mpu_fifo_count >= 1024) {

    // reset so we can continue cleanly
    mpu.resetFIFO();
    mpu_fifo_count = mpu.getFIFOCount();
    Serial.println("MPU: FIFO overflow!");
    // otherwise, check for DMP data ready interrupt (this should happen frequently)

  } else if (mpu_int_status & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

    // wait for correct available data length, should be a VERY short wait
    while (mpu_fifo_count < mpu_packet_size) mpu_fifo_count = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(mpu_fifo_buf, mpu_packet_size);

    // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt)
    mpu_fifo_count -= mpu_packet_size;

#ifdef OUTPUT_TEAPOT_OSC

    static WiFiUDP udp;
    static OSCMessage msg("/imu");
    static Quaternion mpu_quanternion;

    mpu.dmpGetQuaternion(&mpu_quanternion, mpu_fifo_buf);

    //Serial.printf("w, x, y, z = %f, %f, %f, %f\n", (float)mpu_quanternion.w, (float)mpu_quanternion.x, (float)mpu_quanternion.y, (float)mpu_quanternion.z);

    if (WiFi.status() != WL_CONNECTED) {
      return;
    }

    // Send OSC message
    msg.add((float)mpu_quanternion.w);
    msg.add((float)mpu_quanternion.x);
    msg.add((float)mpu_quanternion.y);
    msg.add((float)mpu_quanternion.z);

    udp.beginPacket(config.mpu_udp_ip, config.mpu_udp_port);
    msg.send(udp);
    udp.endPacket();
    msg.empty();

#endif

    /*
      #ifdef OUTPUT_READABLE_QUATERNION
        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, mpu_fifo_buf);
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
      #endif

      #ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, mpu_fifo_buf);
        mpu.dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180 / M_PI);
      #endif

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, mpu_fifo_buf);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, mpu_fifo_buf);
        mpu.dmpGetAccel(&aa, mpu_fifo_buf);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
      #endif

      #ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, mpu_fifo_buf);
        mpu.dmpGetAccel(&aa, mpu_fifo_buf);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        Serial.print("aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);
      #endif

    */

    //    blinkState = !blinkState;
    //    digitalWrite(LED_PIN, blinkState);
  }

}


