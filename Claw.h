#pragma once
#include "IDynamixel.h"
#include "DynamixelSerial.h"

namespace xzj
{

class Sensor
{
  private:
    const double p4=2.2732;
    const double p3 = -31.013;
    const double p2 = 159.11;
    const double p1 = -366.21;
    const double p0 = 322.03;
    const unsigned char pin;
    double threshold;

  public:
    Sensor(unsigned char _pin, double _threshold) : pin(_pin), threshold(_threshold) {}
    double read_load();
    double read_volt();
    void alter_threshold(double);
    bool is_safe();
};

class MyServo : public DynamixelClass
{
  public:
    MyServo(unsigned char _pin, long _baud, long _serial_baud) : pin(_pin), baud(_baud), serial_baud(_serial_baud) {}

    int mov_to(double pos);
    int mov_speed(double pos, double speed);
    double read_pos();
    double read_speed();
    int read_load();
    void stop();
    void stop(double pos);
    void init(int);   //初始化舵机参数
    void launch(); //建立舵机通讯链接,关闭串口
    void shut();   //关闭舵机通讯链接，启动串口
    unsigned long test_com_speed();

    template <typename T>
    void print(T val);

    const unsigned char min_ang = 0;
    const unsigned char max_ang = 100;

  private:
    const unsigned char offset = 60; //建立抽象角度，实际角度，实际角度hex值的映射
    const long baud;
    const long serial_baud;
    const unsigned char pin; //舵机通信接口
};

template <typename T>
void MyServo::print(T val)
{
    shut();
    Serial.println(val);
    launch();
}

class Adaptor
{
  public:
    static int adapt_pos(double pos)
    {
        return (pos / 300) * 0x3ff;
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
        return (static_cast<double>(speed) / 0x3ff) * 6 * 114;
    }
};
} // namespace xzj
