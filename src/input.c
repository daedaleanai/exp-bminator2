#include <string.h>

#include "clock.h"
#include "input.h"
#include "tprintf.h"

#define printf tprintf

// helper for debugging
static void hexdump(size_t len, const uint8_t *ptr) {
	static const char *hexchar = "0123456789abcdef";
	for (size_t i = 0; i < len; ++i) {
		_putchar(' ');
		_putchar(hexchar[ptr[i] >> 4]);
		_putchar(hexchar[ptr[i] & 0xf]);
	}
}

// receiving command packets on usart1
struct CommandBuffer cmdbuf							= {{}, 0};
static enum { RESYNC, MORE, COMPLETE } cmdbuf_state = RESYNC;
static size_t  cmdpacket_size						= 0;

static inline size_t findbyte(uint8_t *buf, size_t buflen, uint8_t b) {
	size_t i = 0;
	for (; i < buflen; ++i)
		if (buf[i] == b)
			break;

	return i;
}

static inline int haspfx(uint8_t *buf, size_t buflen, uint8_t *pfx, size_t pfxlen) {
	if (buflen < pfxlen)
		return 0;

	while (pfxlen--)
		if (*buf++ != *pfx++)
			return 0;

	return 1;
}

static inline uint16_t checksum16(uint8_t *buf, size_t len) {
	uint16_t chk = 0;
	for (size_t i = 0; i < len; ++i)
		chk += buf[i];

	return chk;
}

// return  0,1 for ok, other codes for not ok replies
// and 0xff for when we can't even reply
// see ICD table 'Acknowledgement codes'
static uint8_t checkcmdpacket() {
	uint8_t *buf = cmdbuf.buf;

	if (cmdbuf.head != cmdpacket_size + 2) {
		printf("CMDRX: internal error %u != %u + 2\n", cmdbuf.head, cmdpacket_size);
		return 0xff;
	}

	uint16_t chk  = checksum16(buf, cmdbuf.head - 2);
	uint16_t chk2 = decode_be_uint16(buf + cmdbuf.head - 2);
	if (chk != chk2) {
		printf("CMDRX: invalid packet checksum %u, expected %u\n", chk2, chk);
		return 0xff;
	}
	if (cmdpacket_size < CMDMINSIZE) {
		printf("CMDRX: short packet %d bytes\n", cmdpacket_size);
		return 0x47;
	}

	uint32_t pt = decode_be_uint32(buf);
	if (pt != 0x05050505) {
		printf("CMDRX: invalid packet type %lx\n", pt);
		return 0xff;
	}

	// todo: crc32 over whole packet
	// if bad: return 0x80

	// tag:cmdbuf[4:8] should be 4x same byte
	if ((buf[4] != buf[5]) || (buf[4] != buf[6]) || (buf[4] != buf[7])) {
		return 0x47;
	}

	// must be 0 (read) or 1 (write)
	if (buf[8] & ~1) {
		printf("CMDRX: invalid command %x\n", buf[8]);
		return 0x42;
	}

	uint32_t len = decode_be_uint24(buf + 9);
	if (len > 16) {
		printf("CMDRX: cannot read or write more than 16 bytes, got %lu\n", len);
		return 0x45;
	}

	uint32_t addr = decode_be_uint32(cmdbuf.buf + 12);
	if ((addr & 0xfffffc80) != 0x23000000) {
		printf("CMDRX: invalid address 0x%08lx\n", addr);
		return 0x40;
	}

	// write
	if (buf[8] == 1) {
		switch (addr & 0xffff) {
		case 0x0140:
		case 0x0141:
		case 0x02f0:
		case 0x0210:
		case 0x0372:
		case 0x0374:
		case 0x0375:
			if (len == 1)
				break;
			// fallthrough
		default:
			printf("CMDRX: write packet for %ld bytes at readonly address 0x%08lx\n", len, addr);
			return 0x44;
		}
	}

	// read
	if ((buf[8] == 0) && (cmdpacket_size > CMDMINSIZE)) {
		printf("CMDRX: read packet has %d bytes trailing garbage\n", cmdpacket_size - CMDMINSIZE);
		return 0x46;
	}

	// write
	if ((buf[8] == 1) && (cmdpacket_size != CMDMINSIZE + 4 * ((len + 3) / 4))) {
		printf("CMDRX: write packet for %ld bytes has wrong length %d\n", len, cmdpacket_size - CMDMINSIZE);
		return 0x46;
	}

	return buf[8] & ~1;	 // 0: ok to read, 1: ok to write
}

size_t input_cmdrx(struct MsgQueue *cmdq, struct SPIQ *spiq) {
	switch (cmdbuf_state) {
	case RESYNC:
		while (!haspfx(cmdbuf.buf, cmdbuf.head, "IRON", 4)) {
			size_t start = 1 + findbyte(cmdbuf.buf + 1, cmdbuf.head - 1, 'I');
			memmove(cmdbuf.buf, cmdbuf.buf + start, cmdbuf.head - start);
			cmdbuf.head -= start;
			if (cmdbuf.head < CMDMINSIZE) {
				// go get some more characters until we have 20 again
				return CMDMINSIZE - cmdbuf.head;
			}
		}

		// we have at least 20 bytes starting with 'IRON', next two bytes are packet size
		cmdpacket_size = decode_be_uint16(cmdbuf.buf + 4);
		memmove(cmdbuf.buf, cmdbuf.buf + 6, cmdbuf.head - 6);
		cmdbuf.head -= 6;

		if (cmdpacket_size + 2 > sizeof cmdbuf.buf) {
			printf("CMDRX: invalid packet size %u, ignoring %d bytes\n", cmdpacket_size, cmdbuf.head);
			cmdbuf_state = RESYNC;
			cmdbuf.head	 = 0;
			return CMDMINSIZE;
		}

		cmdbuf_state = MORE;
		// fallthrough

	case MORE:
		if (cmdbuf.head < cmdpacket_size + 2) {
			return 2 + cmdpacket_size - cmdbuf.head;
		}
		// now we have cmdpacket_size + 2 bytes in the buffer
		cmdbuf_state = COMPLETE;
		// fallthrough

	case COMPLETE:
		break;
	}

	// we have a complete packet
	printf("cmdbuf[%d]", cmdbuf.head);
	hexdump(cmdbuf.head, cmdbuf.buf);
	printf("\n");

	uint8_t sts = checkcmdpacket();
	switch (sts) {
	case 0:	 // valid read command
	case 1:	 // valid write command
	{
		uint32_t len  = decode_be_uint24(cmdbuf.buf + 9);
		uint32_t addr = decode_be_uint32(cmdbuf.buf + 12);
		printf("CMDRX %s %ld bytes at address %lx", sts ? "write" : "read", len, addr);

		// schedule the read or write on the spiq
		struct SPIXmit *x = spiq_head(spiq);
		if (x == NULL) {
			printf("CMDRX: spiq overflow\n");
			break;
		}

		x->buf[0] = (addr & 0x7f) | ((sts) ? 0 : 0x80);	// lowest 8 bits: read flag 0x80 + register address
		if (sts) {
			memmove(x->buf + 1, cmdbuf.buf + 16, len);
		} else {
			memset(x->buf + 1, 0xff, len);
		}

		x->ts	  = cycleCount();
		x->tag	  = 0xff000000 | ((uint32_t)cmdbuf.buf[4] << 16) | x->buf[0];
		x->addr	  = (addr >> 8) & 0x3;	// bits 10:9 are the device
		x->status = -1;
		x->len	  = 1 + len;
		spiq_enq_head(spiq);

	} break;

	case 0xff:
		// packet too messed up to reply
		printf("CMDRX: ignoring %d bytes\n", cmdbuf.head);
		break;

	default:
		// queue error response packet on cmdq
		struct Msg *msg = msgq_head(cmdq);
		if (msg == NULL) {
			printf("CMDRX: cmdq overflow\n");
			break;
		}
		msg_reset(msg);
		msg_append32(msg, bytex4(cmdbuf.buf[4]));  // the tag
		msg_append32(msg, bytex4(sts));			   // the status
		msgq_push_head(cmdq);
	}

	cmdbuf_state = RESYNC;
	cmdbuf.head	 = 0;
	return CMDMINSIZE;
}
