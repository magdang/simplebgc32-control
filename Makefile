# simplebgc32-control — standalone SimpleBGC gimbal control. No ROS dependency.
#
#   make            build gimbal_ctl, gimbal_gui and the test binary
#   make test       build and run every test (~40 s; see below)
#   make test-quick same, minus the one slow case (~15 s)
#   make run        build and start the CLI in simulation mode (no hardware)
#   make gui        build and start the status console, then open a browser
#   make probe      build the read-only board prober
#   make clean
#
# Individual suites: test-protocol, test-modules, test-ctl, test-gui, test-page.
#
# The protocol and module suites are C and always run. The other three need
# python3 (and
# node for the page); where an interpreter is missing that suite is SKIPPED
# with a note rather than silently passing — a test that did not run is not a
# test that passed.
#
# test-gui drives the real daemon against a simulated controller on a pty, so
# it needs no hardware and cannot touch a real gimbal. Its slow case waits out
# the 20 s "gimbal never settled" timeout; test-quick skips just that one.

CC       ?= cc
CXX      ?= c++
CFLAGS   ?= -O2 -g -std=c11   -Wall -Wextra -Wpedantic -Wshadow -Wconversion
CXXFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -Wpedantic -Wshadow
CPPFLAGS += -Iinclude -I$(BUILD)
LDLIBS   += -lm

BUILD := build
BIN   := $(BUILD)/gimbal_ctl
GUI   := $(BUILD)/gimbal_gui
PROBE := $(BUILD)/sbgc_probe
TEST  := $(BUILD)/test_sbgc_api
TESTMOD := $(BUILD)/test_modules

PYTHON ?= python3
NODE   ?= node

.PHONY: all test test-quick test-protocol test-modules test-ctl test-gui test-page \
        run gui probe clean

all: $(BIN) $(GUI) $(TEST) $(TESTMOD)

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/sbgc_api.o: src/sbgc_api.c include/sbgc_api.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/gamepad.o: src/gamepad.c include/gamepad.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/sbgc_params.o: src/sbgc_params.c include/sbgc_params.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/sbgc_gui_config.o: src/sbgc_gui_config.c include/sbgc_gui_config.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/httpd.o: src/httpd.c include/httpd.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD)/gimbal_ctl.o: src/gimbal_ctl.cpp include/sbgc_api.h include/gamepad.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

$(BIN): $(BUILD)/gimbal_ctl.o $(BUILD)/sbgc_api.o $(BUILD)/gamepad.o
	$(CXX) $^ -o $@ $(LDLIBS)

# The UI page is compiled into the binary so gimbal_gui is self-contained and
# cannot serve a stale file from disk. Regenerated whenever the HTML changes.
$(BUILD)/web_index.h: web/index.html | $(BUILD)
	@printf '/* Generated from web/index.html by the Makefile. Do not edit. */\n' > $@
	@printf 'static const char WEB_INDEX_HTML[] =\n' >> $@
	@sed -e 's/\\/\\\\/g' -e 's/"/\\"/g' -e 's/^/"/' -e 's/$$/\\n"/' $< >> $@
	@printf ';\n' >> $@
	@printf 'static const unsigned long WEB_INDEX_HTML_LEN = sizeof(WEB_INDEX_HTML) - 1;\n' >> $@

$(BUILD)/gimbal_gui.o: src/gimbal_gui.cpp $(BUILD)/web_index.h \
                       include/sbgc_api.h include/sbgc_params.h \
                       include/sbgc_gui_config.h include/httpd.h include/gamepad.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

$(GUI): $(BUILD)/gimbal_gui.o $(BUILD)/sbgc_api.o $(BUILD)/sbgc_params.o \
        $(BUILD)/sbgc_gui_config.o $(BUILD)/httpd.o $(BUILD)/gamepad.o
	$(CXX) $^ -o $@ $(LDLIBS) -lpthread

$(PROBE): tools/sbgc_probe.c include/sbgc_api.h include/sbgc_params.h \
          $(BUILD)/sbgc_api.o | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(filter %.c %.o,$^) -o $@ $(LDLIBS)

$(TEST): test/test_sbgc_api.c $(BUILD)/sbgc_api.o | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $^ -o $@ $(LDLIBS)

$(TESTMOD): test/test_modules.c $(BUILD)/httpd.o $(BUILD)/sbgc_params.o \
            $(BUILD)/sbgc_gui_config.o $(BUILD)/sbgc_api.o | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(filter %.c %.o,$^) -o $@ $(LDLIBS)

# A suite that could not run leaves a marker here, so the summary below can
# name what was missed instead of hedging unconditionally. Each suite clears
# its own marker before deciding, and every suite target is .PHONY, so the
# markers always describe the run that just happened.
SKIPDIR := $(BUILD)/.skipped
skipped_list = $$(ls $(SKIPDIR) 2>/dev/null | tr '\n' ' ')

test: test-protocol test-modules test-page test-ctl test-gui
	@if [ -n "$(skipped_list)" ]; then \
	    printf "\nSuites that ran passed. SKIPPED (did not run): %s\n" "$(skipped_list)"; \
	else \
	    printf "\nAll suites passed.\n"; \
	fi

# Same coverage minus the case that waits out a 20 s timeout, for the
# edit-build-test loop. Everything else runs.
test-quick: GUI_TEST_ARGS := --quick
test-quick: test-protocol test-modules test-page test-ctl test-gui
	@if [ -n "$(skipped_list)" ]; then \
	    printf "\nSuites that ran passed; slow cases skipped. SKIPPED (did not run): %s\n" "$(skipped_list)"; \
	else \
	    printf "\nAll suites passed; slow cases skipped.\n"; \
	fi

test-protocol: $(TEST)
	@echo "=== protocol (test_sbgc_api.c) ==="
	@./$(TEST)

test-modules: $(TESTMOD)
	@printf "\n=== modules (test_modules.c) ===\n"
	@./$(TESTMOD)

test-ctl: $(BIN)
	@printf "\n=== CLI (test_gimbal_ctl.py) ===\n"
	@mkdir -p $(SKIPDIR) && rm -f $(SKIPDIR)/ctl
	@if command -v $(PYTHON) >/dev/null 2>&1; then \
	    $(PYTHON) test/test_gimbal_ctl.py; \
	else \
	    echo "SKIPPED: $(PYTHON) not found"; touch $(SKIPDIR)/ctl; \
	fi

test-gui: $(GUI)
	@printf "\n=== daemon (test_gimbal_gui.py) ===\n"
	@mkdir -p $(SKIPDIR) && rm -f $(SKIPDIR)/gui
	@if command -v $(PYTHON) >/dev/null 2>&1; then \
	    $(PYTHON) test/test_gimbal_gui.py $(GUI_TEST_ARGS); \
	else \
	    echo "SKIPPED: $(PYTHON) not found"; touch $(SKIPDIR)/gui; \
	fi

# Depends on the page itself, not on a binary: it evaluates web/index.html's
# own <script> block rather than anything compiled.
test-page: web/index.html
	@printf "\n=== page (test_web_page.js) ===\n"
	@mkdir -p $(SKIPDIR) && rm -f $(SKIPDIR)/page
	@if command -v $(NODE) >/dev/null 2>&1; then \
	    $(NODE) test/test_web_page.js; \
	else \
	    echo "SKIPPED: $(NODE) not found"; touch $(SKIPDIR)/page; \
	fi

run: $(BIN)
	@./$(BIN) --simulate

gui: $(GUI)
	@./$(GUI)

probe: $(PROBE)
	@echo "read-only board probe: ./$(PROBE) --port /dev/ttyUSB0"

clean:
	@rm -rf $(BUILD)
