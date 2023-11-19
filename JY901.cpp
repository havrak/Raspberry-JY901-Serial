#include "JY901.h"
#include "JY901_Control.h"
#include <chrono>
#include <iostream>
#include <iterator>

using namespace std;

CJY901::CJY901()
{
	wiringPiSetup();
	lastTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
}

bool CJY901::startI2C(int adress)
{
	i2cadress = adress;
	i2cmode = true;
	fd = wiringPiI2CSetup(adress);
	return fd >= 0;
}

bool CJY901::attach(string device)
{
	fd = serialOpen(device.c_str(), 9600);
	serialPutchar(fd, 0x00);
	serialPutchar(fd, 0x00);
	serialPutchar(fd, 0x00);
	return fd >= 0;
}

bool CJY901::attach(int fdn)
{
	fd = fdn;
	return true;
}

bool CJY901::changeBaudRate(int baud)
{
	setBaudRate(baud);
	serialClose(fd);
	fd = serialOpen("/dev/ttyACM0", baud);
	if (fd < 0)
		return false;
	return true;
}

bool CJY901::writeSerialData(unsigned char* data, int len)
{
	for (int i = 0; i < len; i++) {
		serialPutchar(fd, data[i]);
	}
	return true;
}

void CJY901::writeI2CData(char* data, int len)
{
	cout << "Writing data" << endl;
	for (int i = 0; i < len; i++) {
		cout <<  static_cast<int>(data[i]) << " ";
		wiringPiI2CWrite(fd, data[i]);
	}
	cout << endl;
}

bool CJY901::readSerialData(unsigned char data)
{
	rxBuffer[rxCnt] = data;
	rxCnt++;
	if (rxBuffer[0] != 0x55) { // data start with 0x55 contains what we need
		rxCnt = 0;
		return false;
	}
	if (rxCnt < 11) {
		return false;
	}
	rxCnt = 0; // reset count to 0

	// do sum check to confirm data is not corrupted
	unsigned char sum = 0;
	for (unsigned char cnt = 0; cnt < 10; cnt++)
		sum += rxBuffer[cnt];
	if (sum != rxBuffer[10])
		return false;

	cout << "JY901 | readSerialData | type: " << to_string(rxBuffer[1]) << endl;
	switch (rxBuffer[1]) { // these cases are based on Manual p26,27,28,29 and 30
	case 0x50:
		memcpy(&JY901_data.time, &rxBuffer[2], 8);
		break; // time
	case 0x51:
		memcpy(&JY901_data.acc, &rxBuffer[2], 8);
		break; // acceleration
	case 0x52:
		memcpy(&JY901_data.gyro, &rxBuffer[2], 8);
		break; // angular velocity
	case 0x53:
		memcpy(&JY901_data.angle, &rxBuffer[2], 8);
		break; // angle
	case 0x54:
		memcpy(&JY901_data.mag, &rxBuffer[2], 8);
		break; // magnetic field and temperature
	case 0x55:
		memcpy(&JY901_data.dStatus, &rxBuffer[2], 8);
		break; // D port status

	case 0x56:
		memcpy(&JY901_data.pressure, &rxBuffer[2], 4); // pressure
		memcpy(&JY901_data.altitude, &rxBuffer[6], 4); // altitude
		break;

	case 0x57:
		memcpy(&JY901_data.lon, &rxBuffer[2], 4); // longtitude
		memcpy(&JY901_data.lat, &rxBuffer[6], 4); // latitude
		break;

	case 0x58:
		memcpy(&JY901_data.GPSHeight, &rxBuffer[2], 2);		//
		memcpy(&JY901_data.GPSYaw, &rxBuffer[4], 2);			// GPS data
		memcpy(&JY901_data.GPSVelocity, &rxBuffer[6], 4); //
		break;

	case 0x59:
		memcpy(&JY901_data.quater, &rxBuffer[2], 8);
		break; // quaternion
	case 0x5A:
		memcpy(&JY901_data.GPS_DOP, &rxBuffer[2], 8);
		break; // GPS DOP
	}
	lastTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	return true;
}

bool CJY901::receiveSerialData(void)
{
	bool status = false;
	while (serialDataAvail(fd)) {
		status = CJY901::readSerialData(serialGetchar(fd));
	}
	return status;
} // if data has been retrieved, return true

/* ------------ (JY901 --> Host) functions ------------ */
unsigned short CJY901::getTime(const char* str)
{
	if (strcmp(str, "year") == 0)
		return JY901_data.time.year; // get year
	if (strcmp(str, "month") == 0)
		return JY901_data.time.month; // get month
	if (strcmp(str, "day") == 0)
		return JY901_data.time.day; // get day
	if (strcmp(str, "hour") == 0)
		return JY901_data.time.hour; // get hour
	if (strcmp(str, "minute") == 0)
		return JY901_data.time.minute; // get minute
	if (strcmp(str, "second") == 0)
		return JY901_data.time.second; // get second
	if (strcmp(str, "milisecond") == 0)
		return JY901_data.time.milisecond; // get milisecond
	return 0;
} // getTime()

double CJY901::getTemp()
{
	if (i2cmode)
		JY901_data.mag.temperature = wiringPiI2CReadReg16(fd, JY901_TEMP);
	return JY901_data.mag.temperature / 100.0;
} // getTemp() unit: degree(s) Celsius

double CJY901::getAccX()
{
	if (i2cmode)
		JY901_data.acc.x = wiringPiI2CReadReg16(fd, JY901_AX);
	return JY901_data.acc.x / (32768.0 / 16.0);
} // getAccX() unit: G(gravity)

double CJY901::getAccY()
{
	if (i2cmode)
		JY901_data.acc.y = wiringPiI2CReadReg16(fd, JY901_AY);
	return JY901_data.acc.y / (32768.0 / 16.0);
} // getAccY() unit: G(gravity)

double CJY901::getAccZ()
{
	if (i2cmode)
		JY901_data.acc.z = wiringPiI2CReadReg16(fd, JY901_AZ);
	return JY901_data.acc.z / (32768.0 / 16.0);
} // getAccZ() unit: G(gravity)

double CJY901::getGyroX()
{
	if (i2cmode)
		JY901_data.gyro.x = wiringPiI2CReadReg16(fd, JY901_GX);
	return JY901_data.gyro.x / (32768.0 / 2000.0);
} // getGyroX() unit: degree(s) per second

double CJY901::getGyroY()
{
	if (i2cmode)
		JY901_data.gyro.y = wiringPiI2CReadReg16(fd, JY901_GY);
	return JY901_data.gyro.y / (32768.0 / 2000.0);
} // getGyroY() unit: degree(s) per second

double CJY901::getGyroZ()
{
	if (i2cmode)
		JY901_data.gyro.z = wiringPiI2CReadReg16(fd, JY901_GZ);
	return JY901_data.gyro.z / (32768.0 / 2000.0);
} // getGyroZ() unit: degree(s) per second

/* -- Noticed that The Euler angles' order here is ---- */
/* ----------- Z-Y-X, for more please visit ----------- */
/* --- http://web.mit.edu/2.05/www/Handout/HO2.PDF ---- */
double CJY901::getRoll()
{ // X-axis
	if (i2cmode)
		JY901_data.angle.roll = wiringPiI2CReadReg16(fd, JY901_Roll);
	return JY901_data.angle.roll / (32768.0 / 180.0);
} // getRoll() unit: degree(s)

double CJY901::getPitch()
{ // Y-axis
	if (i2cmode)
		JY901_data.angle.pitch = wiringPiI2CReadReg16(fd, JY901_Pitch);
	return JY901_data.angle.pitch / (32768.0 / 180.0);
} // getPitch() unit: degree(s)

double CJY901::getYaw()
{ // Z-axis
	if (i2cmode)
		JY901_data.angle.yaw = wiringPiI2CReadReg16(fd, JY901_Yaw);
	return JY901_data.angle.yaw / (32768.0 / 180.0);
} // getYaw() unit: degree(s)

double CJY901::getMagX()
{
	if (i2cmode)
		JY901_data.mag.x = wiringPiI2CReadReg16(fd, JY901_HX);
	return JY901_data.mag.x / (32768.0 / 180.0);
} // getMagX()

double CJY901::getMagY()
{
	if (i2cmode)
		JY901_data.mag.y = wiringPiI2CReadReg16(fd, JY901_HY);
	return JY901_data.mag.y / (32768.0 / 180.0);
} // getMagY()

double CJY901::getMagZ()
{
	if (i2cmode)
		JY901_data.mag.z = wiringPiI2CReadReg16(fd, JY901_HZ);
	return JY901_data.mag.z / (32768.0 / 180.0);
} // getMagZ()

/* ------ The port status output depends on its mode. ------ */
/* ----------- For more, please read the manual. ----------- */
short CJY901::getD0Status()
{
	if (i2cmode)
		JY901_data.dStatus.d0 = wiringPiI2CReadReg16(fd, JY901_D0Status);
	return JY901_data.dStatus.d0;
}
short CJY901::getD1Status()
{
	if (i2cmode)
		JY901_data.dStatus.d1 = wiringPiI2CReadReg8(fd, JY901_D1Status);
	return JY901_data.dStatus.d1;
}
short CJY901::getD2Status()
{
	if (i2cmode)
		JY901_data.dStatus.d2 = wiringPiI2CReadReg16(fd, JY901_D2Status);
	return JY901_data.dStatus.d2;
}
short CJY901::getD3Status()
{
	if (i2cmode)
		JY901_data.dStatus.d3 = wiringPiI2CReadReg16(fd, JY901_D3Status);
	return JY901_data.dStatus.d3;
}

int CJY901::getPressure()
{
	if (i2cmode)
		JY901_data.pressure = (wiringPiI2CReadReg16(fd, JY901_PressureH) << 16) | wiringPiI2CReadReg16(fd, JY901_PressureL);
	return JY901_data.pressure;
} // getPressure() unit: Pa

int CJY901::getAltitude()
{
	if (i2cmode)
		JY901_data.altitude = (wiringPiI2CReadReg16(fd, JY901_HeightH) << 16) | wiringPiI2CReadReg16(fd, JY901_HeightL);
	return JY901_data.altitude;
} // getAltitude() unit: cm

/* ------------- According to NMEA8013, ------------ */
/* ------- GPS output format is dd mm.mmmmm, ------- */
/* ----- JY901 output format is ddmm(.)mmmmm, ------ */
/* --------- dd and mm can be calculated ----------- */
/* ---------- by divide(/) and modulo(%) ----------- */
int CJY901::getLon()
{
	if (i2cmode)
		JY901_data.lon = (wiringPiI2CReadReg16(fd, JY901_LonL) << 16) + wiringPiI2CReadReg16(fd, JY901_LonL);

	return JY901_data.lon;
}
int CJY901::getLat()
{
	if (i2cmode)
		JY901_data.lat = (wiringPiI2CReadReg16(fd, JY901_LatL) << 16) + wiringPiI2CReadReg16(fd, JY901_LatL);
	return JY901_data.lat;
}

double CJY901::getGPSH()
{
	if (i2cmode)
		JY901_data.GPSHeight = wiringPiI2CReadReg16(fd, JY901_GPSHeight);
	return JY901_data.GPSHeight / 10.0;
} // get GPS Height, unit: m(meters)

double CJY901::getGPSY()
{
	if (i2cmode)
		JY901_data.GPSYaw = wiringPiI2CReadReg16(fd, JY901_GPSYAW);
	return JY901_data.GPSYaw / 10.0;
} // get GPS Yaw, unit: degree(s)

double CJY901::getGPSV()
{
	if (i2cmode)
		JY901_data.GPSVelocity = (wiringPiI2CReadReg16(fd, JY901_GPSVH) << 16) + wiringPiI2CReadReg16(fd, JY901_GPSVL);
	return JY901_data.GPSVelocity / 1000.0;
} // get GPS Velocity, unit: kilometers per hour

double CJY901::getQuater(const char* str)
{
	if (strcmp(str, "q0") == 0) {
		if (i2cmode)
			JY901_data.quater.q0 = wiringPiI2CReadReg16(fd, JY901_Q0);
		return JY901_data.quater.q0; // get q0
	}
	if (strcmp(str, "q1") == 0) {
		if (i2cmode)
			JY901_data.quater.q1 = wiringPiI2CReadReg16(fd, JY901_Q1);
		return JY901_data.quater.q1; // get q1
	}
	if (strcmp(str, "q2") == 0) {
		if (i2cmode)
			JY901_data.quater.q2 = wiringPiI2CReadReg16(fd, JY901_Q2);
		return JY901_data.quater.q2; // get q2
	}
	if (strcmp(str, "q3") == 0) {
		if (i2cmode)
			JY901_data.quater.q3 = wiringPiI2CReadReg16(fd, JY901_Q3);
		return JY901_data.quater.q3; // get q3
	}
	return 0;
} // getQuater()

/* double CJY901::getDOP(const char* str) */
/* { */
/* 	if (strcmp(str, "sn") == 0) { */
/* 		return JY901_data.GPS_DOP.sn; // get number of satellites */
/* 	} */
/* 	if (strcmp(str, "pdop") == 0) { */
/* 		return JY901_data.GPS_DOP.pdop; // get PDOP */
/* 	} */
/* 	if (strcmp(str, "hdop") == 0) { */
/* 		return JY901_data.GPS_DOP.hdop; // get HDOP */
/* 	} */
/* 	if (strcmp(str, "vdop") == 0) { */
/* 		return JY901_data.GPS_DOP.vdop; // get VDOP */
/* 	} */
/* 	return 0; */
/* } // getDOP() */

milliseconds CJY901::getLastTime()
{
	return lastTime;
} // get last receive time

/* ----------------- Get Raw data if needed ----------------- */
short CJY901::getAccRawX()
{
	if (i2cmode)
		JY901_data.acc.x = wiringPiI2CReadReg16(fd, JY901_AX);
	return JY901_data.acc.x;
}
short CJY901::getAccRawY()
{
	if (i2cmode)
		JY901_data.acc.y = wiringPiI2CReadReg16(fd, JY901_AY);
	return JY901_data.acc.y;
}
short CJY901::getAccRawZ()
{
	if (i2cmode)
		JY901_data.acc.z = wiringPiI2CReadReg16(fd, JY901_AZ);
	return JY901_data.acc.z;
}

short CJY901::getGyroRawX()
{
	if (i2cmode)
		JY901_data.gyro.y = wiringPiI2CReadReg16(fd, JY901_GX);
	return JY901_data.gyro.x;
}
short CJY901::getGyroRawY()
{
	if (i2cmode)
		JY901_data.gyro.y = wiringPiI2CReadReg16(fd, JY901_GY);
	return JY901_data.gyro.y;
}
short CJY901::getGyroRawZ()
{
	if (i2cmode)
		JY901_data.gyro.z = wiringPiI2CReadReg16(fd, JY901_GZ);
	return JY901_data.gyro.z;
}

short CJY901::getMagRawX()
{
	if (i2cmode)
		JY901_data.mag.x = wiringPiI2CReadReg16(fd, JY901_HX);
	return JY901_data.mag.x;
}
short CJY901::getMagRawY()
{
	if (i2cmode)
		JY901_data.mag.x = wiringPiI2CReadReg16(fd, JY901_HY);
	return JY901_data.mag.y;
}
short CJY901::getMagRawZ()
{
	if (i2cmode)
		JY901_data.mag.x = wiringPiI2CReadReg16(fd, JY901_HZ);
	return JY901_data.mag.z;
}
/* ----------------- Raw data Functions end ----------------- */

/* ------------ (Host --> JY901) functions ------------ */
void CJY901::saveConf(char saveFlag)
{
	if (i2cmode) {
		char cmd[2] = { JY901_SAVE, saveFlag };
		writeI2CData(cmd,2);
	} else {
		JY901_SAVECONF[3] = saveFlag;
		writeSerialData(JY901_SAVECONF, 5);
	}
} // save configuration

void CJY901::setCali(char caliFlag)
{
	if (i2cmode) {
		char cmd[2] = { JY901_CALSW, caliFlag };
		writeI2CData(cmd,2);
	} else {
		JY901_SETCALI[3] = caliFlag;
		writeSerialData(JY901_SETCALI, 5);
	}
} // calibration mode

void CJY901::setDir(char dirFlag) // NOTE: no equivalent for i2c
{
	JY901_INSTALL[3] = dirFlag;
	writeSerialData(JY901_INSTALL, 5);
} // set install direction

void CJY901::enterHiber() // NOTE: no equivalent for i2c
{
	writeSerialData(JY901_SLEEP, 5);
} // enter hibernation mode, send again to wake

void CJY901::changeALG(char algFlag) // note: no equivalent for i2c
{
	JY901_ALGAXIS[3] = algFlag;
	writeSerialData(JY901_ALGAXIS, 5);
} // change algorithm

void CJY901::autoCaliGyro(char gyroFlag) // note: no equivalent for i2c
{
	if (i2cmode) {
		char cmd[2] = { JY901_GYROAUTOCALI, gyroFlag };
		writeI2CData(cmd,2);
	} else {
		JY901_GYROAUTO[3] = gyroFlag;
		writeSerialData(JY901_GYROAUTO, 5);
	}
} // auto gyro calibration

void CJY901::confReport() // note: no equivalent for i2c
{
	memcpy(&JY901_RPTCONF[3], &JY901_ctrl.report.confl, 2);
	writeSerialData(JY901_RPTCONF, 5);
} // need to write conf to  JY901_ctrl.report.conf first

void CJY901::setReportRate(char rateFlag)
{
	if (i2cmode) {
		char cmd[2] = { JY901_RRATE, rateFlag };
		writeI2CData(cmd,2);
	} else {

		JY901_RPTRT[3] = rateFlag;
		writeSerialData(JY901_RPTRT, 5);
	}
}

void CJY901::setBaudRate(char baudFlag)
{
	if (i2cmode) {
		char cmd[2] = { JY901_RRATE, baudFlag };
		writeI2CData(cmd,2);
	} else {
		JY901_BAUDRT[3] = baudFlag;
		writeSerialData(JY901_BAUDRT, 5);
	}
}

/* ------To avoid negative value been changed, please use --------- */
/* ------------------- memcpy() to set value of ------------------- */
/* --- JY901_ctrl.aoffset JY901_ctrl.goffset JY901_ctrl.hoffset --- */
/* ----------- For more please read the example folder ------------ */
void CJY901::setAXoffset()
{
	if (i2cmode) {
		char cmd[3] = { JY901_AXOFFSET, JY901_ctrl.aoffset.xl, JY901_ctrl.aoffset.xh };
		writeI2CData(cmd,2);
	} else {
		memcpy(&JY901_AXOFF[3], &JY901_ctrl.aoffset.xl, 2);
		writeSerialData(JY901_AXOFF, 5);
	}
}
void CJY901::setAYoffset()
{
	if (i2cmode) {
		char cmd[3] = { JY901_AYOFFSET, JY901_ctrl.aoffset.yl, JY901_ctrl.aoffset.yh };
		writeI2CData(cmd,2);
	} else {
		memcpy(&JY901_AYOFF[3], &JY901_ctrl.aoffset.yl, 2);
		writeSerialData(JY901_AYOFF, 5);
	}
}
void CJY901::setAZoffset()
{
	if (i2cmode) {
		char cmd[3] = { JY901_AZOFFSET, JY901_ctrl.aoffset.zl, JY901_ctrl.aoffset.zh };
		writeI2CData(cmd,2);
	} else {
		memcpy(&JY901_AZOFF[3], &JY901_ctrl.aoffset.zl, 2);
		writeSerialData(JY901_AZOFF, 5);
	}
}

void CJY901::setGXoffset()
{
	if (i2cmode) {
		char cmd[3] = { JY901_GXOFFSET, JY901_ctrl.goffset.xl, JY901_ctrl.goffset.xh };
		writeI2CData(cmd,3);
	} else {
		memcpy(&JY901_GXOFF[3], &JY901_ctrl.goffset.xl, 2);
		writeSerialData(JY901_GXOFF, 5);
	}
}
void CJY901::setGYoffset()
{
	if (i2cmode) {
		char cmd[3] = { JY901_GYOFFSET, JY901_ctrl.goffset.yl, JY901_ctrl.goffset.yh };
		writeI2CData(cmd,3);
	} else {
		memcpy(&JY901_GYOFF[3], &JY901_ctrl.goffset.yl, 2);
		writeSerialData(JY901_GYOFF, 5);
	}
}
void CJY901::setGZoffset()
{
	if (i2cmode) {
		char cmd[3] = { JY901_GZOFFSET, JY901_ctrl.goffset.zl, JY901_ctrl.goffset.zh };
		writeI2CData(cmd,3);
	} else {
		memcpy(&JY901_GZOFF[3], &JY901_ctrl.goffset.zl, 2);
		writeSerialData(JY901_GZOFF, 5);
	}
}

void CJY901::setHXoffset()
{
	if (i2cmode) {
		char cmd[3] = { JY901_HXOFFSET, JY901_ctrl.hoffset.xl, JY901_ctrl.hoffset.xh };
		writeI2CData(cmd,3);
	} else {
		memcpy(&JY901_HXOFF[3], &JY901_ctrl.hoffset.xl, 2);
		writeSerialData(JY901_HXOFF, 5);
	}
}
void CJY901::setHYoffset()
{
	if (i2cmode) {
		char cmd[3] = { JY901_HYOFFSET, JY901_ctrl.hoffset.yl, JY901_ctrl.hoffset.yh };
		writeI2CData(cmd,3);
	} else {
		memcpy(&JY901_HYOFF[3], &JY901_ctrl.hoffset.yl, 2);
		writeSerialData(JY901_HYOFF, 5);
	}
}
void CJY901::setHZoffset()
{
	if (i2cmode) {
		char cmd[3] = { JY901_HZOFFSET, JY901_ctrl.hoffset.zl, JY901_ctrl.hoffset.zh };
		writeI2CData(cmd,3);
	} else {
		memcpy(&JY901_HZOFF[3], &JY901_ctrl.hoffset.zl, 2);
		writeSerialData(JY901_HZOFF, 5);
	}
}

void CJY901::setD0mode(char modeFlag)
{
	if (i2cmode) {
		char cmd[2] = { JY901_D0MODE, modeFlag };
		writeI2CData(cmd,2);
	} else {
		JY901_D0MODECONF[3] = modeFlag;
		writeSerialData(JY901_D0MODECONF, 5);
	}
}
void CJY901::setD1mode(char modeFlag)
{
	if (i2cmode) {
		char cmd[2] = { JY901_D1MODE, modeFlag };
		writeI2CData(cmd,2);
	} else {
		JY901_D1MODECONF[3] = modeFlag;
		writeSerialData(JY901_D1MODECONF, 5);
	}
}
void CJY901::setD2mode(char modeFlag)
{
	if (i2cmode) {
		char cmd[2] = { JY901_D1MODE, modeFlag };
		writeI2CData(cmd,2);
	} else {
		JY901_D2MODECONF[3] = modeFlag;
		writeSerialData(JY901_D2MODECONF, 5);
	}
}
void CJY901::setD3mode(char modeFlag)
{
	if (i2cmode) {
		char cmd[2] = { JY901_D1MODE, modeFlag };
		writeI2CData(cmd,2);
	} else {
		JY901_D3MODECONF[3] = modeFlag;
		writeSerialData(JY901_D3MODECONF, 5);
	}
}

void CJY901::setD0PWMH()
{
	// JY901_D0PWMH
	memcpy(&JY901_D0PWMHCONF[3], &JY901_ctrl.pwmh.d0l, 2);
	writeSerialData(JY901_D0PWMHCONF, 5);
}
void CJY901::setD1PWMH()
{
	memcpy(&JY901_D1PWMHCONF[3], &JY901_ctrl.pwmh.d1l, 2);
	writeSerialData(JY901_D1PWMHCONF, 5);
}
void CJY901::setD2PWMH()
{
	memcpy(&JY901_D2PWMHCONF[3], &JY901_ctrl.pwmh.d2l, 2);
	writeSerialData(JY901_D2PWMHCONF, 5);
}
void CJY901::setD3PWMH()
{
	memcpy(&JY901_D3PWMHCONF[3], &JY901_ctrl.pwmh.d3l, 2);
	writeSerialData(JY901_D3PWMHCONF, 5);
}

void CJY901::setD0PWMT()
{
	memcpy(&JY901_D0PWMTCONF[3], &JY901_ctrl.pwmt.d0l, 2);
	writeSerialData(JY901_D0PWMTCONF, 5);
}
void CJY901::setD1PWMT()
{
	memcpy(&JY901_D1PWMTCONF[3], &JY901_ctrl.pwmt.d1l, 2);
	writeSerialData(JY901_D1PWMTCONF, 5);
}
void CJY901::setD2PWMT()
{
	memcpy(&JY901_D2PWMTCONF[3], &JY901_ctrl.pwmt.d2l, 2);
	writeSerialData(JY901_D2PWMTCONF, 5);
}
void CJY901::setD3PWMT()
{
	memcpy(&JY901_D3PWMTCONF[3], &JY901_ctrl.pwmt.d3l, 2);
	writeSerialData(JY901_D3PWMTCONF, 5);
}

void CJY901::setIICaddr(int addrFlag)
{
	JY901_IICADDRESS[3] = addrFlag;
	writeSerialData(JY901_IICADDRESS, 5);
}

void CJY901::turnLED(int ledFlag)
{
	JY901_LED[3] = ledFlag;
	writeSerialData(JY901_LED, 5);
}

void CJY901::setGPSrate(int gpsFlag)
{
	JY901_GPSBAUDRATE[3] = gpsFlag;
	writeSerialData(JY901_GPSBAUDRATE, 5);
}
