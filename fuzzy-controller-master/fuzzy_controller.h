#ifndef FUZZY_CONTROLLER_H_
#define FUZZY_CONTROLLER_H_
#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3
#include<iostream>
#include<string>
#include<cmath>
using namespace std;

class Fuzzy_controller
{
public:
	const static int N=7;//定义量化论域模糊子集的个数
private:
	float target;//系统的控制目标
	float actual;//采样获得的实际值
	float e;     //误差
	float e_pre; //上一次的误差
	float de;    //误差的变化率
	float emax;  //误差基本论域上限
	float demax; //误差辩化率基本论域的上限
	float umax;  //输出的上限
	float Ke;    //Ke=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
	float Kde;   //Ke=n/demax,量化论域为[-3,-2,-1,0,1,2,3]
	float Ku;    //Ke=umax/n,量化论域为[-3,-2,-1,0,1,2,3]
	int (*rule)[7] =new int[7][7]{{NB,NB,NM,NM,NS,ZO,ZO},
	                      {NB,NB,NM,NS,NS,ZO,PS},
						  {NM,NM,NM,NS,ZO,PS,PS},
	                      {NM,NM,NS,ZO,PS,PM,PM},
	                      {NS,NS,ZO,PS,PS,PM,PM},
	                      {NS,ZO,PS,PM,PM,PM,PB},
	                      {ZO,ZO,PM,PM,PM,PB,PB}};//模糊规则表

	float *e_mf_paras=new float[21]{-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3}; //误差的隶属度函数的参数
	float *de_mf_paras=new float[21]{-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};//误差的偏差隶属度函数的参数
	float *u_mf_paras=new float[21]{-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3}; //输出的隶属度函数的参数

public:
	Fuzzy_controller(float e_max,float de_max,float u_max);
	~Fuzzy_controller();
	float trimf(float x,float a,float b,float c);          //三角隶属度函数
	float gaussmf(float x,float ave,float sigma);          //正态隶属度函数
	float trapmf(float x,float a,float b,float c,float d); //梯形隶属度函数
	//设置模糊隶属度函数的参数
	void setMf(float *e_mf,float *de_mf,float *u_mf);
	void setRule(int rulelist[N][N]);                          //设置模糊规则
	float realize(float t,float a);              //实现模糊控制
	float test(float e,float ec);
	float test1(float,float);
};

#endif
