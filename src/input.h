#pragma once

#include "msgq.h"
#include "spi.h"

struct CommandBuffer {
	uint8_t buf[128];
	size_t	head;
};

extern struct CommandBuffer cmdbuf;

enum { CMDMINSIZE = 16 };  // shortest command is 16 characters

size_t input_cmdrx(struct MsgQueue *cmdq, struct SPIQ *spiq, struct Scratch *scratch);
