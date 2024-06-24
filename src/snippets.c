snippets

T: 28489 26939 50
P: 36944 -10459 3024 6315 37 -7 9900 -10230 4285
H: 75 378 0 282 50 30
BME read: 0 4
 ff 5a 10 30 80 03 e0 60 d0 t:22008  p:97342256 h:386178
BME read: 0 5
 ff 5a 10 70 80 04 60 60 a7 t:22011  p:97341993 h:383801



bme_writereg(BME_SPI, BME280_REG_CONFIG, BME280_CONFIG_TSB10|BME280_CONFIG_FLT16);
bme_writereg(BME_SPI, BME280_REG_CTRLHUM, BME280_CTRLHUM_H16);
bme_writereg(BME_SPI, BME280_REG_CTRLMEAS, BME280_CTRLMEAS_P16|BME280_CTRLMEAS_T16|BME280_CTRLMEAS_NORMAL);




sample time 24.5
521503895 TRIGGER
521506152 c0 EOC
521509345 c1 EOC
521512500 c2 EOC EOS
531503895 TRIGGER


341503895 TRIGGER
341506121 c0 EOC
341508800 c1 EOC
341511310 c2 EOC EOS
351503895 TRIGGER


11481691228 TRIGGER
11481693650 c0 EOC
11481697047 c1 EOC
11481699862 c2 EOC
11481702515 c3 EOC
11481705168 c4 EOC EOS
adc 938 1368 1408 4095 77
11491691228 TRIGGER


#define CMP_SWAP(i, j) if (a[i] > a[j]) { uint32_t tmp = a[i]; a[i] = a[j]; a[j] = tmp; }

void sort8u32(uint32_t a[8]) {
    CMP_SWAP(0, 1); CMP_SWAP(2, 3); CMP_SWAP(4, 5); CMP_SWAP(6, 7);
    CMP_SWAP(0, 2); CMP_SWAP(1, 3); CMP_SWAP(4, 6); CMP_SWAP(5, 7);
    CMP_SWAP(1, 2); CMP_SWAP(5, 6); CMP_SWAP(0, 4); CMP_SWAP(1, 5);
    CMP_SWAP(2, 6); CMP_SWAP(3, 7); CMP_SWAP(2, 4); CMP_SWAP(3, 5);
    CMP_SWAP(1, 2); CMP_SWAP(3, 4); CMP_SWAP(5, 6);
}




	printf("crc sw: %x\n", crc16_update(0,"IRON", 4));

		CRC.CR = 0;
		crc_cr_set_polysize(&CRC, 1); 16 bits
		CRC.INIT = 0;
		CRC.POL = 0x1021;
		CRC.CR |= CRC_CR_RESET;
		CRC.DR8 = 'I';
		CRC.DR8 = 'R';
		CRC.DR8 = 'O';
		CRC.DR8 = 'N';
		printf("crc hw: %lx\n", CRC.DR32);










		// in silent mode, just discard the packets
		if (0) {
			if (x) {
				spiq_deq_tail(&spiq);
				x = NULL;
			}
			if (ev) {
				spiq_deq_tail(&evq);
				ev = NULL;
			}
			IWDG.KR = 0xAAAA;  // pet the watchdog 
		}
