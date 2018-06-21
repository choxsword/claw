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
    const double p4=0.6699;
    const double p3 = -10.559;
    const double p2 = 62.764;
    const double p1 = -169.1;
    const double p0 = 177.51;
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
    void open_claw(Sensor&);

    void open_claw(double*speed=nullptr);
    void close_claw(double*speed=nullptr);

    template <typename T>
    void print(T val);

    bool is_in_place(const double pos);

    const double min_ang = 0;
    const double max_ang = 95;

  private:
    const double offset = 60; //建立抽象角度，实际角度，实际角度hex值的映射
    const long baud;
    const long serial_baud;
    const unsigned char pin; //舵机通信接口
    double mid_speed=60;

    int wait_position=500;
};

template <typename T>
void MyServo::print(T val)
{
   // Serial.println(val);
}



class Controller{
    public:
    Controller(MyServo& _servo,Sensor & _sensor,Alarm& _alarm):servo(_servo),sensor(_sensor),alarm(_alarm){
    }

    private:
    MyServo & servo;
    Sensor & sensor;
    Alarm & alarm;
    double kp=15;
    double ki=10;

    Fuzzy_controller fuzzy;
    Upstream up_conn;
    public:

	void check();
    void grab_by_p(const double exp_force,const double v0=0);
    void test_p(const double exp_force,const double v0=0);

    void grab_by_pi(const double,const double=0);
    void test_pi(const double,const double=0);

    void grab_by_admit(const double);
    void set_pi(double _kp=10.0,double _ki=8);

    void hold_on(double);
    void launch(double);
     void wait_print_sensor(unsigned long time);
    void wait_print_pos(unsigned long time);

    void test_fuzzy(const double exp_force,const double v0=0);
    void grab_by_fuzzy(const double exp_force,const double v0=0);
};



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
