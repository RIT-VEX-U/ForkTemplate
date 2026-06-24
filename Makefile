.PHONY: all build clean rebuild

ifeq ($(OS),Windows_NT)
PYTHON ?= py -3
else
PYTHON ?= python3
endif
BUILD_PY := build.py

all: build

build:
	@$(PYTHON) $(BUILD_PY) build $(ARGS)

clean:
	@$(PYTHON) $(BUILD_PY) clean $(ARGS)

rebuild:
	@$(PYTHON) $(BUILD_PY) rebuild $(ARGS)
