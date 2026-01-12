# Testing

Build normally using CMake, then
```sh
# Run all tests and get a nicely formatted output
./tests/run_tests.sh

# or get a list of available test suites
./build/bin/test_runner list
# and run them individually (in this case the 'core' test suite)
./build/bin/test_runner core
```

> Tests include private tics headers to do white box testing.

## Debugging with VSCode

After adding the following `.vscode/launch.json`, you will be able to press F5 and enter a name of a test suite to debug it.

```json
{
	"version": "0.2.0",
	"inputs": [
		{
			"id": "testSuiteArg",
			"type": "promptString",
			"description": "Enter the test argument (e.g. 'api') or 'list' to see possible test arguments",
			"default": "all"
		}
	],
	"configurations": [
		{
			"name": "Debug Specific Test Suite",
			"type": "cppdbg",
			"request": "launch",
			"program": "${workspaceFolder}/build/bin/test_runner",
			// This asks you for the argument when you hit F5
			"args": [
				"${input:testSuiteArg}"
			],
			"stopAtEntry": false,
			// Crucial: We must set CWD to binary location for blick to work
			"cwd": "${workspaceFolder}/build/bin",
			"environment": [],
			"externalConsole": false,
			"MIMode": "gdb",
			"setupCommands": [
				{
					"description": "Enable pretty-printing for gdb",
					"text": "-enable-pretty-printing",
					"ignoreFailures": true
				}
			],
			// Build before debugging
			"preLaunchTask": "CMake: build"
		}
	]
}
```
