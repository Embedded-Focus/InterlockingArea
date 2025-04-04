build: build-uno build-esp32

.PHONY: build-uno
build-uno:
	platformio run -e uno

.PHONY: build-esp32
build-esp32:
	platformio run -e esp32
