#SuperSerial Arduino Library for RS232, RS485 communications

##Description
The SuperSerial library wraps RS232 and RS485 communications in a Master / Slave architecture with reliable transport.

A more in-depth summary is provided here: https://zoetrope.io/tech-blog/serial-communication-library-arduinos


##Troubleshooting
If you get a POINTER_REGS spill error use 

ino build -f="-ffunction-sections -fdata-sections -O2 -w"
