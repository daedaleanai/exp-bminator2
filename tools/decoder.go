// Decoder tool for the serial stream produced by the firmware according to the ICD
//
// Sample usage:
//
//	(stty 921600 raw && cat) < /dev/ttyXXX | go run decoder.go
//
// in Monitoring mode, the program produces a summary of the received messages that refreshes
// once a second.
package main

import (
	"bufio"
	"encoding/binary"
	"flag"
	"io"
	"log"
	"os"
)

var (
	fMonitorMode = flag.Bool("m", false, "Monitor mode")
)

func main() {
	flag.Parse()
	var buf [1 << 16]byte
	r := bufio.NewReaderSize(os.Stdin, len(buf))
	for {

		// rescan to the "IRON" tag
		const magic = "IRON"
		_, err := r.ReadSlice(magic[0])
		if err != nil {
			log.Fatal(err)
		}
		var mbuf [len(magic) - 1]byte
		if _, err := io.ReadFull(r, mbuf[:]); err != nil {
			log.Fatal(err)
		}
		if string(mbuf[:]) != magic[1:] {
			continue
		}

		log.Println("Resynced to magic header")

		var framelen uint16
		if err := binary.Read(r, binary.BigEndian, &framelen); err != nil {
			log.Fatal(err)
		}

		if _, err = io.ReadFull(r, buf[:framelen]); err != nil {
			log.Fatal(err)
		}

		log.Printf("Frame %d bytes.", framelen)

		var bsum, chksum uint16
		if err := binary.Read(r, binary.BigEndian, &chksum); err != nil {
			log.Fatal(err)
		}
		for _, v := range buf[:framelen] {
			bsum += uint16(v)
		}

		if chksum != bsum {
			log.Fatalf("Invalid checksum got %d, expected %d.", chksum, bsum)
		}

	}

}
