#include "test.h"

#include <blick_adapter.h>

#include <string.h>

typedef struct {
	const char* arg_name;	  // Command line argument (e.g., "api")
	const char* display_name; // Nice name for the runner (e.g., "API Tests")
	void (*func)(void);
} TestSuite;

// clang-format off
static const TestSuite suites[] = {
	{ "api",  "Public API",     run_api_tests            },
	{ "cd",   "Collision Test", run_collision_test_tests },
	{ "dyn",  "Dynamics",       run_dynamics_tests       },
};
// clang-format on

void cleanup_debug_process(void) {
	BLICK_SHUTDOWN();
}

int main(int argc, char* argv[]) {
	size_t num_suites = sizeof(suites) / sizeof(suites[0]);

	// list: prints all available test suites
	if (argc >= 2 && strcmp(argv[1], "list") == 0) {
		for (size_t i = 0; i < num_suites; ++i) {
			printf("%s;%s\n", suites[i].arg_name, suites[i].display_name);
		}
		return EXIT_SUCCESS;
	}

	// no argument: prints usage information
	if (argc < 2) {
		fprintf(stderr, "Usage: %s <suite_name> | list\n", argv[0]);
		return EXIT_FAILURE;
	}

	const char* target = argv[1];

	// if argument matches a test suite, run this test suite
	// for success exit with 0
	// for failure exit with 1 and prints error information
	for (size_t i = 0; i < num_suites; ++i) {
		atexit(cleanup_debug_process); // close debug viewer on assert (when exit() is called)
		if (strcmp(target, suites[i].arg_name) == 0) {
			suites[i].func();
			return EXIT_SUCCESS;
		}
	}

	fprintf(stderr, "Unknown test suite: %s\n", target);
	return EXIT_FAILURE;
}
