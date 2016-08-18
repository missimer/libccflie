CFLAGS = -Iinclude -MMD -O2 -Wall

# Library related variables
LIB_FOLDER := lib
_LIBRARY := libccflie.a
LIBRARY := $(LIB_FOLDER)/$(_LIBRARY)
LIB_SRC_DIR := src/cflie
_LIB_CFILES := crazyflie.c
_LIB_CFILES += crazyradio.c
_LIB_CFILES += crtppacket.c
_LIB_CFILES += toc.c
LIB_CFILES := $(patsubst %,$(LIB_SRC_DIR)/%,$(_LIB_CFILES))
LIB_OBJS := $(patsubst %.c,%.o,$(LIB_CFILES))

# Example related variables
_EXAMPLES := gui replugging simple hover
EX_SRC_DIR := src/examples
EX_CFILES := $(patsubst %,$(EX_SRC_DIR)/%.c,$(_EXAMPLES))
EX_OBJS := $(patsubst %.c,%.o,$(EX_CFILES))
BIN_FOLDER := bin
EXAMPLES := $(patsubst %,$(BIN_FOLDER)/%,$(_EXAMPLES))
LIBS := -lglfw -lGL -lusb-1.0 -lncurses

OBJS := $(LIB_OBJS) $(EX_OBJS)
DFILES := $(patsubst %.o,%.d,$(OBJS))

CC=gcc
AR=ar

.PHONY: all clean TAGS

all: $(LIBRARY) $(EXAMPLES)

TAGS:
	ctags -eR

$(_LIBRARY): $(LIBRARY)

$(LIBRARY): $(LIB_OBJS)
	@mkdir -p $(dir $(LIBRARY))
	$(AR) rcs $@ $^

$(LIB_OBJS): %.o : %.c
	$(CC) -c $(CFLAGS) $*.c -o $@

$(_EXAMPLES):
	$(MAKE) $(BIN_FOLDER)/$@

$(EXAMPLES): $(BIN_FOLDER)/% : $(EX_SRC_DIR)/%.o $(LIBRARY)
	@mkdir -p $(dir $(EXAMPLES))
	$(CC) -o $@ $^ $(LIBS)

$(EX_OBJS): %.o : %.c
	$(CC) -c $(CFLAGS) $*.c -o $@

clean:
	rm -rf $(BIN_FOLDERS) $(LIB_FOLDER) $(OBJS) $(DFILES)
	find -name "*~" -delete
	rm -rf TAGS

$(OBJS): Makefile

-include $(DFILES)
