all: dht22

dht22: dht22.c
	gcc -g -std=gnu99 -o dht22 dht22.c -lwiringPi

clean:
	rm -f dht22

run:
	sudo ./dht22
