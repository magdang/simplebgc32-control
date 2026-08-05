# simplebgc32-control — standalone SimpleBGC gimbal control. No ROS dependency.
#
#   make            build gimbal_ctl, gimbal_gui and the test binary
#   make test       build and run the protocol tests
#   make run        build and start the CLI in simulation mode (no hardware)
#   make gui        build and start the status console, then open a browser
#   make probe      build the read-only board prober
#   make clean

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

.PHONY: all test run gui probe clean

all: $(BIN) $(GUI) $(TEST)

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

$(PROBE): tools/sbgc_probe.c $(BUILD)/sbgc_api.o | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $^ -o $@ $(LDLIBS)

$(TEST): test/test_sbgc_api.c $(BUILD)/sbgc_api.o | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $^ -o $@ $(LDLIBS)

test: $(TEST)
	@./$(TEST)

run: $(BIN)
	@./$(BIN) --simulate

gui: $(GUI)
	@./$(GUI)

probe: $(PROBE)
	@echo "read-only board probe: ./$(PROBE) --port /dev/ttyUSB0"

clean:
	@rm -rf $(BUILD)
