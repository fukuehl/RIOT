APPLICATION = nativenet
include ../Makefile.tests_common

BOARD_WHITELIST := native

USEMODULE += nativenet
USEMODULE += transceiver

include $(RIOTBASE)/Makefile.include

FORCE:
	touch main.c

sender: CFLAGS += -DSENDER
sender: APPLICATION = nativenet_sender
sender: FORCE all
