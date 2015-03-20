#include "Arduino.h"
#include <SuperSerial.h>

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
 #define DEBUG_PRINT(x)  Serial.print (x)
 #define DEBUG_PRINTLN(x) Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif


SuperSerial::SuperSerial()
{
	reset();
	_localStation = 0;
	currentlyTXing = false;
	dataPosition = 0;
}

//Takes the MAX489 RE and DE pins, RE is enabled low, DE is enabled high
//For the MAX489, baud rate should be fine up to about 1000000, where slew rate
//limiting starts to become a problem. If a non rate limited chip is used, and
//termination is correct, higher baud rates should be possible.
void SuperSerial::begin(HardwareSerial * serialInterface, unsigned long baudRate, byte station, byte master, byte re, byte de){
	_Serial = serialInterface;
	de_pin = de;
	re_pin = re;
	pinMode(re, OUTPUT);
	pinMode(de, OUTPUT);
	digitalWrite(re, 0);
	digitalWrite(de, 0);
	_Serial->begin(baudRate);
	_localStation = station;
	masterAddress = master;


	state = WAITING_FOR_HEADER;
}

void SuperSerial::printErrorStats() {
	Serial.println(F("D, Errors:"));
	Serial.print(F("D, Malformed Packets: "));
	Serial.println(error_stats.malformed_packets);
	Serial.print(F("D, Bad Checksums: "));
	Serial.println(error_stats.bad_checksums);
	Serial.print(F("D, Poll Timeouts: "));
	Serial.println(error_stats.poll_timeouts);
}

void SuperSerial::send(byte station, byte command, byte len, void *data) {
	send(station, command, len, data, PACKET_CMD);
}


void SuperSerial::sendResponse(byte station, byte command, byte len, void *data) {
	send(station, command, len, data, PACKET_RESP);
}

void SuperSerial::sendCompleteResponse(byte station, byte command, byte len, void *data) {
	send(station, command, len, data, PACKET_COMPLETE);
}

void SuperSerial::broadcast(byte command, byte len, void *data) {
	send(0, command, len, data, PACKET_BROADCAST);
}

void SuperSerial::startTX() {
	digitalWrite(de_pin, 1);
	//digitalWrite(re_pin, 1);
	currentlyTXing = true;
}

void SuperSerial::stopTX() {
	_Serial->flush();
	digitalWrite(de_pin, 0);
	//digitalWrite(re_pin, 0);
	currentlyTXing = false;
}

bool SuperSerial::ping(byte station) {
	send(station, 0, 0, NULL, PACKET_PING);

	while (!process(station, 'A')) {}

	return true;

}


void SuperSerial::send(byte station, byte command, byte len, void *data, typeOfPacket packetType) {

	/*DEBUG_PRINT(F("Sending packet to station: "));
	DEBUG_PRINT(station);
	DEBUG_PRINT(F(" with command: "));
	DEBUG_PRINT(command);
	DEBUG_PRINT(F(" and packet type: "));
	DEBUG_PRINTLN(packetType);
	DEBUG_PRINT(F(" and data[0]: "));
	DEBUG_PRINTLN(((int *)data)[0]);*/

	startTX();

	byte checksum = 0;

	for (byte i = 0; i < 5; i++)
		_Serial->write(SOH);

	_Serial->write(station);
	checksum += station;

	_Serial->write(_localStation);
	checksum += _localStation;

	_Serial->write(command);
	checksum += command;

	_Serial->write(len);
	checksum += len;

	_Serial->write(packetType);
	checksum += packetType;

	_Serial->write(STX);

	for (byte i = 0; i < len; i++)	{
		_Serial->write(((byte*)data)[i]);
		checksum += ((byte *) data)[i];
	}

	_Serial->write(ETX);

	_Serial->write(checksum);

	_Serial->write(EOT);

	stopTX();
}



bool SuperSerial::checkValidChecksum() {
	byte checksum = 0;
	checksum = packet.sender + packet.destinationAddress + packet.command + packet.dataLength + packet.packetType;
	for (byte i = 0; i < packet.dataLength; i++) {
		checksum += packet.data[i];
	}
	return (packet.checksum == checksum);
}

void SuperSerial::reset() {
	state = WAITING_FOR_HEADER;
	dataPosition = 0;
	memset(&packet, 0, sizeof packet);
}

bool SuperSerial::process() {
	return process(NULL, NULL);
}

bool SuperSerial::process(byte stationArg, byte commandArg)
{
	byte rx_byte;

	while (_Serial->available()) {
		rx_byte = _Serial->read();
		switch(state) {
			case WAITING_FOR_HEADER: {
				//Rotate the buffer
				memcpy(&header[0], &header[1], 6);
				header[6] = rx_byte;
				if ((header[6] == STX) && (header[0] == SOH) && ((header[1] == _localStation) || (header[1] == 0))) {
					//We have a header with target address matching our own
					packet.complete = false;
					packet.destinationAddress = header[1];
					packet.sender = header[2];
					packet.command = header[3];
					packet.dataLength = header[4];
					//TODO: Nasty illegal cast.
					packet.packetType = static_cast<typeOfPacket>(header[5]);
					state = WAITING_FOR_DATA;

					DEBUG_PRINTLN("D, Packet for us");
				}

			} break;
			case WAITING_FOR_DATA: {
				//We've got the header now. Get the data if there is any
				if (packet.dataLength > 0) {
					packet.data[dataPosition++] = rx_byte;
					if (dataPosition == packet.dataLength) {
						state = WAITING_FOR_ETX;
						dataPosition = 0;
					}
					break;
				}
				else {
					state = WAITING_FOR_CHECKSUM;

				}
			} break;
			case WAITING_FOR_ETX: {
					//Data finished get ETX.
				if (rx_byte == ETX) {
					state = WAITING_FOR_CHECKSUM;
				}
				else {
					error_stats.malformed_packets++;
					Serial.print(F("D,Malformed packet rx != ETX was : "));
					Serial.println(rx_byte);
					Serial.print(F("D, Packet is (disregard after data)"));

					printPacket();

					reset();
				}
			} break;
			case WAITING_FOR_CHECKSUM: {
				packet.checksum = rx_byte;
				state = WAITING_FOR_EOT;
			} break;
			case WAITING_FOR_EOT: {
				//Make sure we get EOT
				if (rx_byte == EOT) {
					if (checkValidChecksum()) {
						packet.complete = true;
						DEBUG_PRINTLN(F("D, Valid Packet received!"));

						if ((packet.packetType == PACKET_PONG) && (commandArg) && (packet.sender == stationArg)) {
							return true;
						}

						//If the packet's an ACK and this function was called
						//with arguments (from a blocking function), and the
						//command matches, then set ACKResponse.
						if ((packet.packetType == PACKET_RESP || packet.packetType==PACKET_COMPLETE)
						    && (commandArg) &&(packet.sender == stationArg)
							&& (packet.command == commandArg)) {
							ACKResponse.sender = packet.sender;
							ACKResponse.command = packet.command;
							ACKResponse.dataLength = packet.dataLength;
							ACKResponse.packetType = packet.packetType;
							memcpy(ACKResponse.data, packet.data, packet.dataLength);
							reset();

							return true;
						}
						else if ((packet.packetType == PACKET_PING) && (packet.destinationAddress == _localStation)) {
							//Serial.println(F("Sending PONG"));
							send(packet.sender, 0, 0, NULL, PACKET_RESP);
						}
						else {
							//Serial.println(F("Calling Callback"));
							executeCallback();
							reset();
							return true;
						}
					}
					else {
						Serial.println(F("D, Discard packet, checksum failure"));
						error_stats.bad_checksums++;
						reset();
						return false;
					}
				}
				else {
					error_stats.malformed_packets++;
					Serial.print(F("D, Malformed packet, rx != EOT, was:"));
					Serial.println(rx_byte);
					reset();
					return false;
				}
			} break;
		}
	}
	return false;
}



void SuperSerial::executeCallback() {
	registeredCallbackFunction(packet.sender, packet.command, packet.dataLength, packet.data, packet.packetType);
}

void SuperSerial::registerCallback(callbackFunction func) {
	registeredCallbackFunction = func;
}

bool SuperSerial::poll(ACKResponse_t * packetBuffer, byte station, byte command, byte len, void *data, uint16_t packetResponseTimeout, uint16_t packetPollInterval) {

	/*DEBUG_PRINT(F("D, Polling station: "));
	DEBUG_PRINT(station);
	DEBUG_PRINT(F(" with command: "));
	DEBUG_PRINTLN(command);*/

	//Send the command
	send(station, command, len, data, PACKET_CMD);

	long lastRecv = millis();
	long lastSend = millis();

	while(millis()-lastRecv < packetResponseTimeout) {
		if (process(station,command)){
			//There was a packet.
			DEBUG_PRINT(F("D, Received packet from: "));
			DEBUG_PRINT(ACKResponse.sender);
			lastRecv = millis();
			memcpy(packetBuffer, &ACKResponse, sizeof(ACKResponse_t));
			return true;
		}
		//Poll if we've not polled for interval and didn't receive a response
		if (millis() - lastSend > packetPollInterval) {
			/*DEBUG_PRINT(F("D, After timeout, Polling station: "));
			DEBUG_PRINT(station);
			DEBUG_PRINT(F("D, With command: "));
			DEBUG_PRINT(command);*/
			send (station, command, len, data, PACKET_CMD);
			lastSend = millis();
		}
	}
	Serial.println(F("D, No response in timeout"));
	error_stats.poll_timeouts++;
	ACKResponse.packetType = PACKET_NONE;
	//Copy the structure into the buffer (deep-copy)
	memcpy(packetBuffer, &ACKResponse, sizeof(ACKResponse_t));
	return false;

}


bool SuperSerial::pollUntilComplete(ACKResponse_t * packetBuffer, byte station, byte command, byte len, void *data, uint16_t packetResponseTimeout, uint16_t packetPollInterval, uint16_t pollCompleteTimeout) {

	//Poll the desired station until it returns a completed response

	long lastSend = millis();
	long startSend = millis();

	while ((packetBuffer->packetType != PACKET_COMPLETE)) {

		if ((millis() - lastSend) > packetPollInterval) {
			bool pollOkay = poll(packetBuffer, station,command,len,data, packetResponseTimeout, packetPollInterval);
			lastSend = millis();

			if (!pollOkay) {
				Serial.println("D, Returning, bad poll");
				return false;
			}
		}

		if  ((millis()-startSend) > pollCompleteTimeout) {
			Serial.println("D, Poll timed out");
			return false;
		}
	}

	return true;

}

void SuperSerial::printPacket() {
	Serial.println(packet.sender);
	Serial.println(packet.destinationAddress);
	Serial.println(packet.command);
	Serial.println(packet.dataLength);
	Serial.println(packet.packetType);
	Serial.println(packet.checksum);
}
