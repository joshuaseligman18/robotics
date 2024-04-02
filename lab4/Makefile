default: compile upload

compile:
	arduino-cli compile *.ino -e

upload:
	arduino-cli upload --input-dir build/pololu-a-star.avr.a-star32U4

monitor:
	arduino-cli monitor -p /dev/cu.usbmodem14401
