#!/bin/bash

if [ "${1}" == "" ]; then
	echo "usage: sendcmd.sh <pulse ms>"
	exit
fi
SEQ=1
while(true); do  
	read -n 1 CHAR
	if [ "$CHAR" == "u" ]; then
		echo UP
		echo pin 1 0 $1 $SEQ | socat -su - udp-sendto:255.255.255.255:7892,broadcast
	fi
	if [ "$CHAR" == "d" ]; then
		echo DOWN
		echo pin 0 0 $1 $SEQ| socat -su - udp-sendto:255.255.255.255:7892,broadcast
	fi
	SEQ=$(( $SEQ + 1 ))
done
