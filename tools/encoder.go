package main

import (
	"encoding/binary"
	"hash/crc32"
	"log"
	"math/bits"
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
		0x00000080, // read 128 bytes
		0x23000100, // from gyro address space
	}

	packet = append(packet, ^crc(packet))

	if v := crc(packet); v != 0 {
		log.Println("crc not zero:", crc)
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

// TODO match CoaxPress Standard v 2.2 section 9.2.2.2
// cant seem to get this to work https://go.dev/play/p/kmGLmvN-H6k
func crc(vv []uint32) uint32 {
	r := uint32(0xffffffff)
	var b [4]byte
	for _, v := range vv {
		binary.LittleEndian.PutUint32(b[:], bits.Reverse32(v))
		r = crc32.Update(r, crc32.IEEETable, b[:4])
	}
	binary.LittleEndian.PutUint32(b[:], bits.Reverse32(r))
	return binary.BigEndian.Uint32(b[:])
}
