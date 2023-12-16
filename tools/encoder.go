package main

import (
	"encoding/binary"
	"math/rand"
	"os"
)

func main() {

	tag := rand.Uint32() & 0xff
	tag = tag<<8 | tag
	tag = tag<<16 | tag

	packet := []uint32{
		0x05050505, // tagged command
		tag,        // random tag
		0x00000010, // read 16 bytes
		0x23000100, // from gyro address space
	}

	var chk16 uint16
	for _, v := range packet {
		chk16 += uint16(v&0xff) + uint16(v>>8&0xff) + uint16(v>>16&0xff) + uint16(v>>24&0xff)
	}

	os.Stdout.WriteString("IRON")
	binary.Write(os.Stdout, binary.BigEndian, uint16(4*len(packet)))
	binary.Write(os.Stdout, binary.BigEndian, packet)
	binary.Write(os.Stdout, binary.BigEndian, uint16(chk16))

}
