#!/bin/bash

while(true); do  
	read -n 1 CHAR
	if [ "$CHAR" == "u" ]; then
		echo UP
		echo pin 0 0 $1 | socat -su - udp-sendto:255.255.255.255:7892,broadcast
	fi
	if [ "$CHAR" == "d" ]; then
		echo DOWN
		echo pin 1 0 $1 | socat -su - udp-sendto:255.255.255.255:7892,broadcast
	fi
done
