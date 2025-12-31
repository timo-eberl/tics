#!/bin/bash

# This script gets a list of available test suites and runs each.

# Configuration
TEST_BIN="./build/tests/test_runner"

# Colors
COL_RED="\033[0;31m"
COL_GREEN="\033[0;32m"
COL_RESET="\033[0m"

if [ ! -f "$TEST_BIN" ]; then
	echo "Error: Test binary not found at $TEST_BIN"
	exit 1
fi

echo "------------------------------------------------------------"
echo " TICS TEST RUNNER"
echo "------------------------------------------------------------"

OVERALL_SUCCESS=true

# Read suites from binary: "arg_name;Display Name"
while IFS=';' read -r CMD NAME; do
	# Skip empty lines if any
	[ -z "$CMD" ] && continue

	# Print Name
	printf "%s " "$NAME"

	# Print Dots
	PAD_LEN=$((50 - ${#NAME}))
	if [ $PAD_LEN -lt 0 ]; then PAD_LEN=0; fi
	printf '%*s' "$PAD_LEN" | tr ' ' '.'

	# Run the test
	OUTPUT=$($TEST_BIN "$CMD" 2>&1)
	EXIT_CODE=$?

	if [ $EXIT_CODE -eq 0 ]; then
		printf " [ ${COL_GREEN}PASS${COL_RESET} ]\n"
	else
		printf " [ ${COL_RED}FAIL${COL_RESET} ]\n"
		echo "$OUTPUT"
		OVERALL_SUCCESS=false
	fi

done < <($TEST_BIN list)

echo "------------------------------------------------------------"

if [ "$OVERALL_SUCCESS" = true ]; then
	echo -e "RESULT: ${COL_GREEN}All tests passed successfully.${COL_RESET}"
	exit 0
else
	echo -e "RESULT: ${COL_RED}Some tests failed.${COL_RESET}"
	exit 1
fi
