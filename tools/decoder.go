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

func toCelsius(milliK uint32) float64 {
	return float64(int32(milliK) - 273150) / 1000
}

func toPascal(milliPa uint32) float64 {
	return float64(milliPa) / 1000
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
	return fmt.Sprintf("%.6f %14s: %v", float64(m.Ts)/80000000, label(m), [2]float64{toCelsius(m.Val[0]), toPascal(m.Val[1])})
}
func (m *HUMIDMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %v", float64(m.Ts)/80000000, label(m), m.Val)
}
func (m *TEMPMsg) String() string {
	return fmt.Sprintf("%.6f %14s: %v", float64(m.Ts)/80000000, label(m), [2]float64{toCelsius(m.Val[0]), toCelsius(m.Val[1])})
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

	if *fMonitorMode {
		fmt.Println(HOME, CLRBOS)
	}

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

		var csum uint16
		if err := binary.Read(r, binary.BigEndian, &csum); err != nil {
			log.Fatal(err)
		}
		bsum := crc_update(0, buf[:framelen])

		if csum != bsum {
			log.Printf("Invalid crc16 got %016b, expected %016b.", csum, bsum)
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
			if word0 == 0 {
				// padding
				//todo: verify remainder is zero
				break
			}

			if word0 == 0x06060606 {
				log.Printf("Command response[%d]: tag:0x%02x status:0x%02x", framelen, buf[4], buf[8])
				if framelen >= 16 {
					log.Printf("    data[0x%02x]: % 02x", buf[12:16], buf[16:framelen])
				}
				break
			}

			sz := (word0 >> 16) - 4
			if sz > 32 {
				log.Printf("Invalid message length 0x%x (%d)", sz, sz)
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
			//fmt.Println(CLRBOS)

			for k, v := range seen {
				if now-v.T > 2*time.Second {
					delete(seen, k)
				} else {
					seen[k].Count = 0
				}

			}

		} // messages in frame
	} // forever
}

var crc_table = [256]uint16{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
}

func crc_update(crc uint16, data []byte) uint16 {
	for _, v := range data {
		crc = crc_table[byte(crc>>8)^v] ^ (crc << 8)
	}
	return crc
}
