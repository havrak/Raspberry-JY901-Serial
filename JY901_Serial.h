#ifndef _JY901_SERIAL_H_
#define _JY901_SERIAL_H_

#include <wiringPi.h>
#include <wiringSerial.h>


#include <cstring>
#include <string>
#include <chrono>

using namespace std::chrono;

using namespace std;

class CJY901 {

 public:
  CJY901();                           // construct function


	bool attach(string device);
  bool attach(int fd);   							// bind serial connection
	bool writeSerialData(unsigned char* data, int len);
  bool readSerialData(unsigned char data);  // process recieved data
  bool receiveSerialData(void);       // recieve data from serial port
	bool changeBaudRate(int newBaud);

  /* ------------ (JY901 --> Host) functions ------------ */
  unsigned short getTime(const char*);  // get time
  double getTemp();               // get temperature
  double getAccX();               // get X-axis acceleration
  double getAccY();               // get Y-axis acceleration
  double getAccZ();               // get Z-axis acceleration
  double getGyroX();              // get X-axis angular velocity
  double getGyroY();              // get Y-axis angular velocity
  double getGyroZ();              // get Z-axis angular velocity
  double getRoll();               // get X-axis(Roll) angle
  double getPitch();              // get Y-axis(Pitch) angle
  double getYaw();                // get Z-axis(Yaw) angle
  double getMagX();               // get X-axis magnetic field
  double getMagY();               // get Y-axis magnetic field
  double getMagZ();               // get Z-axis magnetic field
  short getD0Status();          // get D0 Status
  short getD1Status();          // get D1 Status
  short getD2Status();          // get D2 Status
  short getD3Status();          // get D3 Status
  int getPressure();          // get pressure(JY-901B)
  int getAltitude();          // get altitude(JY-901B)
  int getLon();               // get lontitude
  int getLat();               // get latitude
  double getGPSH();               // GPS height
  double getGPSY();               // GPS speed angle
  double getGPSV();               // GPS speed
  double getQuater(const char*);  // get quaternion
  double getDOP(const char*);     // get GPS DOP
  milliseconds getLastTime();    // get last receive time
  short getAccRawX();           // get X-axis raw acceleration data
  short getAccRawY();           // get Y-axis raw acceleration data
  short getAccRawZ();           // get Z-axis raw acceleration data
  short getGyroRawX();          // get X-axis raw angular velocity data
  short getGyroRawY();          // get Y-axis raw angular velocity data
  short getGyroRawZ();          // get Z-axis raw angular velocity data
  short getMagRawX();           // get X-axis raw magnetic field data
  short getMagRawY();           // get Y-axis raw magnetic field data
  short getMagRawZ();           // get Z-axis raw magnetic field data

  /* ------------ (Host --> JY901) functions ------------ */
  void saveConf(int);      // save configuration
  void setCali(int);       // calibration mode
  void setDir(int);        // set install direction
  void enterHiber();       // enter hibernation or wake
  void changeALG(int);     // change algorithm
  void autoCaliGyro(int);  // enable auto gyro calibration
  void confReport();       // configure report contents
  void setReportRate(int);
  void setBaudRate(int);

  void setAXoffset();
  void setAYoffset();
  void setAZoffset();

  void setGXoffset();
  void setGYoffset();
  void setGZoffset();

  void setHXoffset();
  void setHYoffset();
  void setHZoffset();

  void setD0mode(int);
  void setD1mode(int);
  void setD2mode(int);
  void setD3mode(int);

  void setD0PWMH();
  void setD1PWMH();
  void setD2PWMH();
  void setD3PWMH();

  void setD0PWMT();
  void setD1PWMT();
  void setD2PWMT();
  void setD3PWMT();

  void setIICaddr(int);
  void turnLED(int);
  void setGPSrate(int);

  struct {
    struct {
      unsigned char confl;
      unsigned char confh;
    } report;

    struct {
      char xl;
      char xh;
      char yl;
      char yh;
      char zl;
      char zh;
    } aoffset;

    struct {
      char xl;
      char xh;
      char yl;
      char yh;
      char zl;
      char zh;
    } goffset;

    struct {
      char xl;
      char xh;
      char yl;
      char yh;
      char zl;
      char zh;
    } hoffset;

    struct {
      unsigned char d0l;
      unsigned char d0h;
      unsigned char d1l;
      unsigned char d1h;
      unsigned char d2l;
      unsigned char d2h;
      unsigned char d3l;
      unsigned char d3h;
    } pwmh;

    struct {
      unsigned char d0l;
      unsigned char d0h;
      unsigned char d1l;
      unsigned char d1h;
      unsigned char d2l;
      unsigned char d2h;
      unsigned char d3l;
      unsigned char d3h;
    } pwmt;

  } JY901_ctrl;

 private:
  int fd = -1;

  milliseconds lastTime;
  unsigned char rxBuffer[12] = {0};
  unsigned char rxCnt = 0;

  struct {
    struct {
      unsigned char year;
      unsigned char month;
      unsigned char day;
      unsigned char hour;
      unsigned char minute;
      unsigned char second;
      unsigned short milisecond;
    } time;

    struct {
      short x;
      short y;
      short z;
      short temperature;
    } acc;

    struct {
      short x;
      short y;
      short z;
      short temperature;
    } gyro;

    struct {
      short roll;
      short pitch;
      short yaw;
      short temperature;
    } angle;

    struct {
      short x;
      short y;
      short z;
      short temperature;
    } mag;

    struct {
      short d0;
      short d1;
      short d2;
      short d3;
    } dStatus;

    int pressure;
    int altitude;  // JY-901B

    int lon;
    int lat;
    short GPSHeight;
    short GPSYaw;
    int GPSVelocity;

    struct {
      short q0;
      short q1;
      short q2;
      short q3;
    } quater;

    struct {  // DOP stands for Dilution of Precision
      short sn;
      short pdop;
      short hdop;
      short vdop;
    } GPS_DOP;

  } JY901_data;
};
#endif
