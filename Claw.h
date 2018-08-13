#pragma once
//#define _CLAW_DEBUG_
#include "IDynamixel.h"
#include "DynamixelSerial.h"
#include "Alarm.h"
#include "fuzzy.h"
#include "Upstream.h"
namespace xzj
{
	typedef unsigned long time_t;
	class Sensor
	{
	private:
		const double p4 = 0.6699;
		const double p3 = -10.559;
		const double p2 = 62.764;
		const double p1 = -169.1;
		const double p0 = 177.51;
		const unsigned char pin;
		double threshold;

	public:
		Sensor(unsigned char _pin, double _threshold) : pin(_pin), threshold(_threshold) {}
		double read_load();
		int readForceSafe();
		double read_volt();
		void alter_threshold(double);
		bool is_safe();
	};

	struct Info {
		int iSpeed;
		int iAng;
		int iForce;
	};


	class MyServo : public DynamixelClass
	{
	public:
		MyServo(unsigned char _pin, long _baud, long _serial_baud);
		int mov_to(double pos);
		int mov_speed(double pos, double speed);

		//返回抽象角度
		double read_pos();
		double read_speed();
		int read_load();

		int readLoadSafe() {
			int val;
			while (val =DynamixelClass::readLoad()<0);
			return val;
		}

		int readPostion() {
			return readPositionSafe();
		}
		int readSpeed() {
			return readSpeedSafe();
		}

		int readPositionSafe() {
			int val;
			while ((val =DynamixelClass::readPosition()) < 2);
			return val;
		}

		int readSpeedSafe() {
			int val;
			while ((val =DynamixelClass::readSpeedOrigin())<0);
			return val;
		}

		void stop();
		void stop(double pos);

		void init(int);   //初始化舵机参数
		void launch(); //建立舵机通讯链接,关闭串口
		void shut();   //关闭舵机通讯链接，启动串口
		unsigned long test_com_speed();
		void open_claw(Sensor&);

		void open_claw(double*speed = nullptr);
		void close_claw(double*speed = nullptr);

		template <typename T>
		void print(T val);

		bool is_in_place(const double pos);
		bool is_in_place(const int& ideal_pos);

		const double min_ang = 0;
		const double max_ang = 95;
		int iMinAng;
		int iMaxAng;
	private:
		const double offset = 60; //建立抽象角度，实际角度，实际角度hex值的映射,实际角度=抽象角度+Offset
		const long baud;
		const long serial_baud;
		const unsigned char pin; //舵机通信接口
		double mid_speed = 60;

		int wait_position = 500;
	};

	template <typename T>
	void MyServo::print(T val)
	{
		// Serial.println(val);
	}


	static const double speed_convert[4] ={ -0.000028983, 0.011139, -1.0675, 2.8469 };

	class Adaptor
	{
	public:
		static int adapt_pos(double pos)
		{
			return (pos / 300) * 0x3ff;
		}

		static double speed2end_speed(double speed, double ang)
		{
			double idx = 0;
			for (int i = 0; i<4; ++i)
			{
				idx += speed_convert[i] * pow(ang, 3 - i);
			}
			return speed * idx / 100.0;
		}
	
		static double end_speed2servo_speed(double speed, double ang) {

			double idx = speed_convert[0] * pow(ang, 3) + speed_convert[1] * pow(ang, 2) + speed_convert[2] * pow(ang, 1) + speed_convert[3];
			return (speed * 100.0)/idx;

		}
		static double format_i2d(int input) {
			return (double)input / 1023 * 100.0;
		}

		static int adapt_speed(double speed)
		{
			return speed / (6 * 114) * 0x3ff;
		}
		static int adapt_temp(double temp);
		static int adapt_torque(double torq);
		static double rec_pos(int pos)
		{
			return (static_cast<double>(pos) / 0x3ff) * 300;
		}
		static double rec_speed(int speed)
		{
			return (static_cast<double>(speed) / 0x3ff) * 6 * 114;//返回度/秒
		}
		static double sensor_i2d(int force) {
			return static_cast<double>(force) / 1023.0*10.0;
		}
	};
} // namespace xzj
