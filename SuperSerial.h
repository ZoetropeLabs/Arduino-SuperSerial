/*
 * Packet structure is as follows:
 * SOH SOH SOH SOH SOH (5 bytes)
 * Destination address (1 byte)
 * Sender address (1 byte)
 * Command (1 byte)
 * Data Length in bytes(1 byte)
 * Packet type (1 byte) (CMD, RESP, COMPLETE, BROADCAST, PING, PONG, NONE)
 * STX (1 byte)
 * Data (len bytes)
 * ETX (1 byte)
 * Checksum (1 byte) (sum of packet excluding special chars  mod 255)
 * EOT (1 byte)
 */

//Packet delimitors
#define SOH 1
#define STX 2
#define ETX 3
#define EOT 4

#define MAX_COMMANDS 10

#define PACKET_CMD 0b00000001
#define PACKET_RESP 0b00000010
#define PACKET_COMPLETE 0b00000100
#define PACKET_BROADCAST 0b00001000
#define PACKET_PING 0b00010000
#define PACKET_PONG 0b01000000
#define PACKET_NONE 0b10000000


#define PACKET_RESPONSE_TIMEOUT 500//150
#define PACKET_POLL_INTERVAL 300//50

//70 seconds
#define POLL_COMPLETE_TIMEOUT 70000

//Data buffer is predefined, if you need more than 10bytes of data per packet, increase this
#define DATA_BUFFER_SIZE 10



typedef byte typeOfPacket;

typedef struct ACKResponse_t {
	byte sender;
	byte command;
	byte dataLength;
	byte data[DATA_BUFFER_SIZE];
	byte packetType;
};


class SuperSerial{

	private:

		byte masterAddress;
		byte currentlyTXing;
		byte re_pin;
		byte de_pin;
		HardwareSerial * _Serial;

	public:
		/*
		 * Constructor
		 */
		SuperSerial();

		/*
		 * Struct to hold cumulative error statistics.
		 */
		struct error_stats {
			int malformed_packets;
			int bad_checksums;
			int poll_timeouts;
		} error_stats;

		/*
		 * Print the current error statistics
		 */
		void printErrorStats();
		/*
		 * Print the current packet
		 */
		void printPacket();

		/*
		 * The response buffer
		 */
		ACKResponse_t ACKResponse;

		/*
		 * Defines the format of the callback function.
		 * The callback will be passed a number of parameters about the
		 * received packet.
		 * @param The sender
		 * @param The command
		 * @param The data length
		 * @param The data pointer
		 * @param The packet type
		 */
		typedef void(*callbackFunction)(byte, byte, byte, void *, typeOfPacket);


		/*
		 * Initialise serial.
		 *
		 * @param	serialDevice The hardware serial device to use
		 * @param	baudRate The baud rate to use on the serial device. With
		 *			the MAX489, 100000 is roughly the upper limit due to the
		 *			internal slew rate limiting. With proper termination and no
		 *			slew rate limiting, this could be higher.
		 * @param	station The address of this device
		 * @param	master The address of the master device (this would be the
		 *			same as station if this is the master device.
		 * @param	re The pin used to enable the receiver
		 * @param	de The pin used to enable the driver
		 */
		void begin(HardwareSerial * serialDevice, unsigned long baudRate, byte station, byte master, byte re, byte de);

		/*
		 * Send a packet. Default packet type is CMD
		 * @param	station The address of the target device
		 * @param	command The command to send
		 * @param	len	The length of data included in the packet
		 * @param	*data A pointer to the data (should be automatically cast
		 *			to a void pointer.
		 */
		void send(byte station, byte command, byte len, void *data);
		/*
		 * Send a packet with a specific type
		 * @param	station The address of the target device
		 * @param	command The command to send
		 * @param	len	The length of data included in the packet
		 * @param	*data A pointer to the data (should be automatically cast
		 *			to a void pointer.
		 * @param	packetType The packet type, e.g. command, response etc.
		 */
		void send(byte station, byte command, byte len, void *data, typeOfPacket packetType);
		/*
		 * Send a Response packet
		 * @param	station The address of the target device
		 * @param	command The command to send
		 * @param	len	The length of data included in the packet
		 * @param	*data A pointer to the data (should be automatically cast
		 *			to a void pointer.

		 */
		void sendResponse(byte station, byte command, byte len, void *data);
		/*
		 * Send a complete response packet.
		 * @param	station The address of the target device
		 * @param	command The command to send
		 * @param	len	The length of data included in the packet
		 * @param	*data A pointer to the data (should be automatically cast
		 *			to a void pointer.
		 */
		void sendCompleteResponse(byte station, byte command, byte len, void *data);


		/* Broadcast sends a CMD packet to station 0x0 which is picked up by all
		 * slaves. No response should be returned by the slaves after
		 * a broadcast.
		 * @param	command The command to send
		 * @param	len	The length of data included in the packet
		 * @param	*data A pointer to the data (should be automatically cast
		 *			to a void pointer.

		 */
		void broadcast(byte command, byte len, void *data);

		/*
		 * Start transmitting. Sets the rx/tx enable pins
		 */
		void startTX();
		/*
		 * Stop transmitting. Sets the rx/tx enable pins
		 */
		void stopTX();

		/*
		 * Process should be called as often as possible, it picks up packets
		 * from the buffer and decodes them, calling the callback if it receives
		 * a command.
		 * @return {bool} indicating whether a packet was received
		 */
		bool process();


		/*
		 * If given a station and command to listen for, process assumes it is
		 * being called from a blocking function and will only pick up matching
		 * packets.
		 * @return {bool} indicating whether a packet was received
		 */
		bool process(byte stationArg, byte commandArg);

		/*
		 * Resets the state machine used for process and internal packet
		 */
		void reset();

		/*
		 * Checks the current packet to make sure the checksum is valid
		 * @return {bool} indicating whether the checksum is valid
		 */
		bool checkValidChecksum();

		void executeCallback();

		/*
		 * Registers a callback function to be executed when a packet is
		 * received. If this is a master device, you will likely want to
		 * discard the packet since poll and pollUntilComplete will handle
		 * response and complete packets. For slave devices, the callback would
		 * generally include a switch statement for each viable command.
		 */
		void registerCallback(callbackFunction func);

		/*
		 * Ping a station, function will return once a pong packet has been
		 * received. Generally used as a utility function to check a slave is
		 * alive and find out the latency on the link.
		 * @return {bool}
		 */
		bool ping(byte station);

		/*
		 * Sends a command to another station, waits for a reponse and then returns.
		 * Does NOT repeat the poll until a COMPLETE response is received
		 * Use pollUntilComplete for that functionality
		 * If the poll times out then the returned packet will have a type of
		 * PACKET_NONE
		 */
		bool poll(ACKResponse_t * packetBuffer, byte station, byte command, byte len, void * data, uint16_t packetResponseTimeout=PACKET_RESPONSE_TIMEOUT, uint16_t packetPollInterval=PACKET_POLL_INTERVAL);

		/*
		 * Blocking function which will only return once the slave has
		 * completed the designated command (by sending a PACKET_COMPLETE
		 * packet).
		 * If there is any point where a slave does not respond within
		 * \a pollCompleteTimeout, the function will return PACKET_NONE.
		 * Otherwise it will only return PACKET_COMPLETE.
		 */

		 bool pollUntilComplete(ACKResponse_t * packetBuffer, byte station, byte command, byte len, void *data, uint16_t packetResponseTimeout=PACKET_RESPONSE_TIMEOUT, uint16_t packetPollInterval=PACKET_POLL_INTERVAL, uint16_t pollCompleteTimeout=POLL_COMPLETE_TIMEOUT);

		callbackFunction registeredCallbackFunction;

	private:
		byte _localStation;
		enum {WAITING_FOR_HEADER, WAITING_FOR_DATA, WAITING_FOR_ETX, WAITING_FOR_CHECKSUM, WAITING_FOR_EOT} state;



		/*
		 * Internal structure used to store the current packet
		 */
		struct packet_t {
			bool complete;
			byte sender;
			byte destinationAddress;
			byte command;
			byte dataLength;
			typeOfPacket packetType;
			byte data[DATA_BUFFER_SIZE];
			byte checksum;
		} packet;


		/*
		 * Struct to store callbacks for each command
		 */
		struct commands_t {
			byte commandCode;
			callbackFunction callback;
		} commands[MAX_COMMANDS];

		byte dataPosition;
		byte header[7];

};
