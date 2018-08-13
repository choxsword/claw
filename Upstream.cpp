#include "Upstream.h"
#include "Arduino.h"

#define sendData(args)  (Serial.write(args))    // Write Over Serial
#define availableData() (Serial.available())    // Check Serial Data Available
#define readData()      (Serial.read())         // Read Serial Data
#define peekData()      (Serial.peek())         // Peek Serial Data
#define TX_DELAY_TIME				100      // Este parametro depende de la velocidad de transmision - pero pueden ser cambiados para mayor velocidad.
#define PROTOCOL_FIRST   0x11
#define PROTOCOL_SECOND  0x22
using namespace xzj;

void Upstream::check() {

	rcv_cmd = Command::Undefined;
	if (availableData() < 6)
		return;
		//Serial.write(readData());
	int Incoming_Byte = readData();
	if ((Incoming_Byte == PROTOCOL_FIRST) && ((unsigned char)peekData() ==(unsigned char) PROTOCOL_SECOND)) {
		readData();                            // Start Bytes
		int cmd = readData();
		int Low_Byte = readData();            // Position Bytes
		int High_Byte = readData();
		int check_sum = readData();
		if (((~(cmd + Low_Byte + High_Byte)) & 0xff) != check_sum)
		{
			return;
		}
		int Long_Byte = High_Byte << 8;
		rcv_data = Long_Byte + Low_Byte;
		rcv_cmd = static_cast<Command>(cmd);
		feed_back(Command::RcvCmd, 0);//反馈收到指令，不用对应具体指令，就算上位机重传，底下也有下位机保证。
		return;
	}
	return;
}

//反馈的val可以携带16位信息
void Upstream::feed_back(const Command& status,int val){
    sendData(PROTOCOL_FIRST);
    sendData(PROTOCOL_SECOND);
    int cstatus=static_cast<int>(status);
	int lowb=val&0xff;
	int highb=val>>8;
	sendData(cstatus);
	sendData(lowb);
	sendData(highb);
	int Checksum=(~(cstatus+lowb+highb))&0xff;
    sendData(Checksum);
}

