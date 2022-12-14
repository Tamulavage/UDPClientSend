# UDPClientSend
Designed, built, tested on Arduino Nano rp2040 Connect 

Sketch to send 2 buttons, POT, and angle of gryoscope over wifi using UDP.

Connections: 
 - Button A: Pin 4
 - Button B: Pin 3
 - POT : A1

 Change local IP address to the IP address that will recieve the UDP. Add your wifi infomration in a file called secrets.h
 
 This was tested out sending to a pythonapplication on a RPI
