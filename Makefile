# Makefile for host-side unit tests
# Requires Unity framework

CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -I./include -I./unity/src
LDFLAGS = -lm

# Unity framework path (adjust if needed)
UNITY_SRC = unity/src/unity.c
UNITY_INC = unity/src

# Test executables
TESTS = filter_test mapping_test

.PHONY: all clean test run-tests

all: $(TESTS)

# Filter test
filter_test: test/filter_test.c $(UNITY_SRC)
	$(CC) $(CFLAGS) -I$(UNITY_INC) -o $@ $^ $(LDFLAGS)

# Mapping test
mapping_test: test/mapping_test.c $(UNITY_SRC)
	$(CC) $(CFLAGS) -I$(UNITY_INC) -o $@ $^ $(LDFLAGS)

# Run all tests
test: $(TESTS)
	@echo "Running filter tests..."
	./filter_test
	@echo ""
	@echo "Running mapping tests..."
	./mapping_test
	@echo ""
	@echo "All tests completed!"

run-tests: test

# Clean build artifacts
clean:
	rm -f $(TESTS) *.o
	rm -f filter_test.exe mapping_test.exe

# Install Unity (if not present)
unity:
	@if [ ! -d "unity" ]; then \
		echo "Downloading Unity framework..."; \
		git clone --depth 1 https://github.com/ThrowTheSwitch/Unity.git unity; \
	else \
		echo "Unity framework already installed"; \
	fi

help:
	@echo "Available targets:"
	@echo "  all        - Build all test executables"
	@echo "  test       - Build and run all tests"
	@echo "  clean      - Remove build artifacts"
	@echo "  unity      - Download Unity framework"
	@echo "  help       - Show this help message"
