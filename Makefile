.PHONY: all build clean rebuild upload run stop

ifeq ($(OS),Windows_NT)
PYTHON ?= py -3
else
PYTHON ?= python3
endif
BUILD_PY := build.py

ifneq ($(filter run,$(MAKECMDGOALS)),)
RUN_SLOT := $(filter-out run,$(MAKECMDGOALS))
ifneq ($(filter-out 1 2 3 4 5 6 7 8,$(RUN_SLOT)),)
$(error Usage: make run [1-8])
endif
.PHONY: 1 2 3 4 5 6 7 8
1 2 3 4 5 6 7 8:
	@:
endif

all: build

build:
	@$(PYTHON) $(BUILD_PY) build $(ARGS)

clean:
	@$(PYTHON) $(BUILD_PY) clean $(ARGS)

rebuild:
	@$(PYTHON) $(BUILD_PY) rebuild $(ARGS)

upload:
	@$(PYTHON) $(BUILD_PY) upload $(ARGS)

run:
	@$(PYTHON) $(BUILD_PY) run $(RUN_SLOT)

stop:
	@$(PYTHON) $(BUILD_PY) stop
