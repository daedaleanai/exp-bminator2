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
	"bytes"
	"encoding/binary"
	"flag"
	"fmt"
	"io"
	"log"
	"os"
	"sort"
	"strings"
	"time"
)

type MsgID uint16

const (
	EVENTID_ID0           MsgID = 0x8003 // [2]uint32 appinfo.revision, serial number [2]
	EVENTID_ID1           MsgID = 0x8004 // [2]uint32 Serial number[1][0]
	EVENTID_BARO          MsgID = 0x8020 // [2]uint32 temperature[milliK] pressure[milliPa] bme280
	EVENTID_HUMID         MsgID = 0x8021 // humidity bme280 [TODO]
	EVENTID_TEMP          MsgID = 0x8022 // [2]uint32 temperature[milliK] (bmi085 accellerometer, stm32 microcontroller)
	EVENTID_SHUTTER_OPEN  MsgID = 0x8023 // uint64 counter
	EVENTID_SHUTTER_CLOSE MsgID = 0x8025 // uint64 counter

	// [4]int16 xyz_ (i.e. padded to 8 bytes)
	EVENTID_ACCEL_3G       MsgID = 0x8032 // RANGE_3G  = 0x00,
	EVENTID_ACCEL_6G       MsgID = 0x8033 // RANGE_6G  = 0x01,
	EVENTID_ACCEL_12G      MsgID = 0x8034 // RANGE_12G  = 0x02,
	EVENTID_ACCEL_24G      MsgID = 0x8035 // RANGE_24G = 0x03,
	EVENTID_GYRO_125DEG_S  MsgID = 0x8038 // BMI085_GYRO_RANGE_125DEG_S  = 0x04,
	EVENTID_GYRO_250DEG_S  MsgID = 0x8039 // BMI085_GYRO_RANGE_250DEG_S  = 0x03,
	EVENTID_GYRO_500DEG_S  MsgID = 0x803a // BMI085_GYRO_RANGE_500DEG_S  = 0x02,
	EVENTID_GYRO_1000DEG_S MsgID = 0x803b // BMI085_GYRO_RANGE_1000DEG_S = 0x01,
	EVENTID_GYRO_2000DEG_S MsgID = 0x803c // BMI085_GYRO_RANGE_2000DEG_S = 0x00,
)

func (m MsgID) String() string { return fmt.Sprintf("%04x", uint16(m)) }

type Pld4Uint16 struct {
	Ts  uint64
	Val [4]uint16
}

type Pld2Uint32 struct {
	Ts  uint64
	Val [2]uint32
}

type PldUint64 struct {
	Ts  uint64
	Val uint64
}

type ID0Msg Pld2Uint32
type ID1Msg Pld2Uint32
type BAROMsg Pld2Uint32
type HUMIDMsg Pld2Uint32
type TEMPMsg Pld2Uint32
type SHUTTER_OPENMsg PldUint64
type SHUTTER_CLOSEMsg PldUint64

type ACCEL_3GMsg Pld4Uint16
type ACCEL_6GMsg Pld4Uint16
type ACCEL_12GMsg Pld4Uint16
type ACCEL_24GMsg Pld4Uint16
type GYRO_125DEG_SMsg Pld4Uint16
type GYRO_250DEG_SMsg Pld4Uint16
type GYRO_500DEG_SMsg Pld4Uint16
type GYRO_1000DEG_SMsg Pld4Uint16
type GYRO_2000DEG_SMsg Pld4Uint16

func scalexyz(v [4]uint16, s float64) [3]float64 {
	s /= (1 << 15)
	return [3]float64{float64(int16(v[0])) * s, float64(int16(v[1])) * s, float64(int16(v[2])) * s}
}

func label(v interface{}) string {
	return strings.TrimPrefix(strings.TrimSuffix(fmt.Sprintf("%T", v), "Msg"), "*main.")
}

func (m *ID0Msg) String() string {
	return fmt.Sprintf("%.6f %14s: %v", float64(m.Ts)/80000000, label(m), m.Val)
}
func (m *ID1Msg) String() string {
	return fmt.Sprintf("%.6f %14s: %v", float64(m.Ts)/80000000, label(m), m.Val)
}
func (m *BAROMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %v", float64(m.Ts)/80000000, label(m), m.Val)
}
func (m *HUMIDMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %v", float64(m.Ts)/80000000, label(m), m.Val)
}
func (m *TEMPMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %v", float64(m.Ts)/80000000, label(m), m.Val)
}
func (m *SHUTTER_OPENMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %v", float64(m.Ts)/80000000, label(m), m.Val)
}
func (m *SHUTTER_CLOSEMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %v", float64(m.Ts)/80000000, label(m), m.Val)
}

// note: BMI088 is 3,6,12,24G instead of 2,4,8,16
func (m *ACCEL_3GMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %.3f ", float64(m.Ts)/80000000, label(m), scalexyz(m.Val, 3))
}
func (m *ACCEL_6GMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %.3f ", float64(m.Ts)/80000000, label(m), scalexyz(m.Val, 6))
}
func (m *ACCEL_12GMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %.3f ", float64(m.Ts)/80000000, label(m), scalexyz(m.Val, 8))
}
func (m *ACCEL_24GMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %.3f ", float64(m.Ts)/80000000, label(m), scalexyz(m.Val, 24))
}
func (m *GYRO_125DEG_SMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %.3f ", float64(m.Ts)/80000000, label(m), scalexyz(m.Val, 125))
}
func (m *GYRO_250DEG_SMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %.3f ", float64(m.Ts)/80000000, label(m), scalexyz(m.Val, 250))
}
func (m *GYRO_500DEG_SMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %.3f ", float64(m.Ts)/80000000, label(m), scalexyz(m.Val, 500))
}
func (m *GYRO_1000DEG_SMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %.3f ", float64(m.Ts)/80000000, label(m), scalexyz(m.Val, 1000))
}
func (m *GYRO_2000DEG_SMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %.3f ", float64(m.Ts)/80000000, label(m), scalexyz(m.Val, 2000))
}

func newMsg(word0 uint32) interface{} {
	if (word0 >> 16) != 20 {
		return nil
	}
	switch MsgID(word0 & 0xffff) {
	case EVENTID_ID0:
		return &ID0Msg{}
	case EVENTID_ID1:
		return &ID1Msg{}
	case EVENTID_BARO:
		return &BAROMsg{}
	case EVENTID_HUMID:
		return &HUMIDMsg{}
	case EVENTID_TEMP:
		return &TEMPMsg{}
	case EVENTID_SHUTTER_OPEN:
		return &SHUTTER_OPENMsg{}
	case EVENTID_SHUTTER_CLOSE:
		return &SHUTTER_CLOSEMsg{}
	case EVENTID_ACCEL_3G:
		return &ACCEL_3GMsg{}
	case EVENTID_ACCEL_6G:
		return &ACCEL_6GMsg{}
	case EVENTID_ACCEL_12G:
		return &ACCEL_12GMsg{}
	case EVENTID_ACCEL_24G:
		return &ACCEL_24GMsg{}
	case EVENTID_GYRO_125DEG_S:
		return &GYRO_125DEG_SMsg{}
	case EVENTID_GYRO_250DEG_S:
		return &GYRO_250DEG_SMsg{}
	case EVENTID_GYRO_500DEG_S:
		return &GYRO_500DEG_SMsg{}
	case EVENTID_GYRO_1000DEG_S:
		return &GYRO_1000DEG_SMsg{}
	case EVENTID_GYRO_2000DEG_S:
		return &GYRO_2000DEG_SMsg{}
	}
	return nil
}

type rec struct {
	T     time.Duration
	Count int
	Msg   interface{}
}

const (
	HOME    = "\033[H"
	CLREOL  = "\033[K"
	CLRBOS  = "\033[J"
	BOLDON  = "\033[1m"
	BOLDOFF = "\033[0m"
)

var (
	fMonitorMode = flag.Bool("m", false, "Monitor mode")
)

func main() {
	flag.Parse()
	var buf [1 << 16]byte
	r := bufio.NewReaderSize(os.Stdin, len(buf))

	seen := map[MsgID]*rec{}
	start := time.Now()
	last := 0 * time.Second
	ticker := time.Tick(time.Second)
	resyncing := false
	frameno := 0
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
			resyncing = true
			continue
		}

		if resyncing {
			log.Println("Resynced to magic header")
		}
		resyncing = false
		now := time.Now().Sub(start)

		var framelen uint16
		if err := binary.Read(r, binary.BigEndian, &framelen); err != nil {
			log.Fatal(err)
		}

		if _, err = io.ReadFull(r, buf[:framelen]); err != nil {
			log.Fatal(err)
		}

		frameno++
		if !*fMonitorMode {
			log.Printf("Frame %d %d bytes %f fps.", frameno, framelen, float64(time.Second)/float64(now-last))
		}
		last = now

		var bsum, chksum uint16
		if err := binary.Read(r, binary.BigEndian, &chksum); err != nil {
			log.Fatal(err)
		}
		for _, v := range buf[:framelen] {
			bsum += uint16(v)
		}

		if chksum != bsum {
			log.Printf("Invalid checksum got %d, expected %d.", chksum, bsum)
		}

		//fmt.Printf("frame [%d] %x\n", framelen, buf[:framelen])

		b := bytes.NewBuffer(buf[:framelen])
		for {
			var word0 uint32
			err := binary.Read(b, binary.BigEndian, &word0)
			if err == io.EOF {
				break
			}

			if err != nil {
				log.Println(err)
				break
			}
			//log.Printf("word0: %08x\n", word0)
			sz := (word0 >> 16) - 4
			if sz > 32 {
				log.Printf("Invalid message length %d", sz)
				break
			}
			var pldbuf [32]byte
			if _, err := io.ReadFull(b, pldbuf[:sz]); err != nil {
				log.Fatal(err)
			}
			//fmt.Printf("[%d] %x\n", sz, pldbuf[:sz])

			msg := newMsg(word0)
			if msg == nil {
				log.Printf("Invalid message type %x len %d", word0&0xffff, word0>>16)
				break
			}
			if err := binary.Read(bytes.NewBuffer(pldbuf[:sz]), binary.BigEndian, msg); err != nil {
				log.Println(err)
				break
			}

			if !*fMonitorMode {
				fmt.Println(msg)
				continue
			}

			id := MsgID(word0 & 0xffff)
			if seen[id] == nil {
				seen[id] = &rec{now, 0, msg}
			} else {
				seen[id].T = now
				seen[id].Count++
				seen[id].Msg = msg
			}

			select {
			case t := <-ticker:
				now = t.Sub(start)
			default:
				continue
			}

			fmt.Println(HOME, now, CLREOL)

			var ids []int
			for k, _ := range seen {
				ids = append(ids, int(k))
			}
			sort.Ints(ids)

			for _, v := range ids {
				id := MsgID(v)
				//				fmt.Printf("%v %12v %4d %v%v\n", id, seen[id].T, seen[id].Count, seen[id].Msg, CLREOL)
				fmt.Printf("%v %4d %v%v\n", id, seen[id].Count, seen[id].Msg, CLREOL)
			}
			fmt.Println(CLRBOS)

			for k, v := range seen {
				if now-v.T > 2*time.Second {
					delete(seen, k)
				} else {
					seen[k].Count = 0
				}

			}
		}
	}
}
