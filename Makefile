.PHONY: all test upload monitor
	
test:
	pio test -e d1_mini --verbose

upload:
	pio run -e d1_mini -t upload --verbose

monitor:
	pio device monitor
