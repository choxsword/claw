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


Command Upstream::check() {
	if (availableData() < 6)
		return Command::Undefined;

		//Serial.write(readData());
	int Incoming_Byte = readData();
	if ((Incoming_Byte == PROTOCOL_FIRST) && ((unsigned char)peekData() ==(unsigned char) PROTOCOL_SECOND)) {
		readData();                            // Start Bytes
		int cmd = readData();
		int Low_Byte = readData();            // Position Bytes
		int High_Byte = readData();
		int check_sum = readData();
		if (((~(cmd + Low_Byte + High_Byte)) & 0xff) != check_sum)
			return Command::Undefined;
	
		int Long_Byte = High_Byte << 8;
		Long_Byte = Long_Byte + Low_Byte;
		Command rcv_cmd = static_cast<Command>(cmd);
		feed_back(rcv_cmd);
		return rcv_cmd;
	}

	return Command::Undefined;
}

void Upstream::feed_back(const Command& status){
    sendData(PROTOCOL_FIRST);
    sendData(PROTOCOL_SECOND);
    int cstatus=static_cast<int>(status);
	sendData(cstatus);
	int Checksum=(~cstatus)&0xff;
    sendData(Checksum);
}