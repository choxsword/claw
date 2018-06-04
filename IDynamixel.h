#pragma once

enum class Direction:char{Open,Close};

class IDynamixel{
public:
	virtual     void begin(long baud, unsigned char directionPin)=0;
	virtual 	void begin(long baud)=0;
	virtual 	void end(void)=0;
	virtual 	int reset()=0;
	virtual 	int ping()=0;
	virtual 	int setID( unsigned char newID)=0;
	virtual 	int setBD( long baud)=0;
	virtual 	int move( int Position)=0;
	virtual 	int moveSpeed( int Position, int Speed)=0;
	virtual 	int setEndless(bool Status)=0;
	virtual 	int turn( bool SIDE, int Speed)=0;
	virtual 	int moveRW( int Position)=0;
	virtual 	int moveSpeedRW( int Position, int Speed)=0;
	virtual 	void action(void)=0;
	virtual 	int setTempLimit( unsigned char Temperature)=0;
	virtual 	int setAngleLimit( int CWLimit, int CCWLimit)=0;
	virtual 	int setVoltageLimit( unsigned char DVoltage, unsigned char UVoltage)=0;
	virtual 	int setMaxTorque( int MaxTorque)=0;
	virtual 	int setSRL( unsigned char SRL)=0;
	virtual 	int setRDT( unsigned char RDT)=0;
	virtual 	int setLEDAlarm( unsigned char LEDAlarm)=0;
	virtual 	int setShutdownAlarm( unsigned char SALARM)=0;
	virtual 	int setCMargin( unsigned char CWCMargin, unsigned char CCWCMargin)=0;
	virtual 	int setCSlope( unsigned char CWCSlope, unsigned char CCWCSlope)=0;
	virtual 	int setPunch( int Punch)=0;
	virtual 	int moving()=0;
	virtual 	int lockRegister()=0;
	virtual 	int RWStatus()=0;
	virtual 	int readTemperature()=0;
	virtual 	int readVoltage()=0;
	virtual 	int readPosition()=0;
	virtual 	int readSpeed()=0;
	virtual 	int readLoad()=0;
	virtual 	int torqueStatus( bool Status)=0;
	virtual 	int ledStatus( bool Status)=0;

};