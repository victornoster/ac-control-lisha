#include <RFID.hpp>

RFID::RFID() {
}

//Constructor prepare the CS pin for SPI communication
RFID::RFID(GPIO_TypeDef* _csPort, uint16_t _csPin){
    csPort = _csPort;
    csPin = _csPin;
} //End constructor

/**
 * Writes a uint8_t to the specified register in the RFID chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void RFID::RFID_WriteRegister(uint8_t reg, uint8_t value) {
	GPIO_Output(csPort, csPin, 0); //select rfid sensor
	uint8_t aux = reg & 0x7E;
	SPI_Transfer(aux);// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	SPI_Transfer(value);
	GPIO_Output(csPort, csPin, 1); // Release rfid sensor
} // End RFID_WriteRegister()

void RFID::RFID_WriteRegister(uint8_t reg, uint8_t count, uint8_t *values){
    MX_SPI1_Init(); //calls the init function of spi.h
    GPIO_Output(csPort, csPin, 0); //select rfid sensor
    SPI_Transfer(reg & 0x7E);
    for (uint8_t index = 0; index < count; index++) {
		SPI_Transfer(values[index]);
	}
	GPIO_Output(csPort, csPin, 1); // Release rfid sensor
}

uint8_t RFID::RFID_ReadRegister(uint8_t reg){
		uint8_t value;
	    GPIO_Output(csPort, csPin, 0); //select rfid sensor			// Select slave
	    SPI_Transfer(0x80 | (reg & 0x7E));			// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
		value = SPI_Receive();					// Read the value back.
		SPI_Transfer(0);	//Send 0 to stop reading.
		GPIO_Output(csPort, csPin, 1); // Release rfid sensor
		return value;
}

void RFID::RFID_ReadRegister(uint8_t reg, uint8_t count, uint8_t *values, uint8_t rxAlign){
    if (count == 0) {
		return;
	}
	//Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" uint8_ts from register."));
	uint8_t address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	uint8_t index = 0;							// Index in values array.
    GPIO_Output(csPort, csPin, 0);          //select rfid sensor
	count--;								// One read is performed outside of the loop
	SPI_Transfer(address);					// Tell RFID which address we want to read
	while (index < count) {
		if (index == 0 && rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
			// Create bit mask for bit positions rxAlign..7
			uint8_t mask = 0;
			for (uint8_t i = rxAlign; i <= 7; i++) {
				mask |= (1 << i);
			}
			// Read value and tell that we want to read the same address again.
			uint8_t value = SPI_Receive();
			SPI_Transfer(address);
			// Apply mask to both current value of values[0] and the new data in value.
			values[0] = (values[index] & ~mask) | (value & mask);
		}
		else { // Normal case
			values[index] = SPI_Receive();	// Read value and tell that we want to read the same address again.
			SPI_Transfer(address);
		}
		index++;
	}
	values[index] = SPI_Receive();	// Read the final uint8_t
	SPI_Transfer(0);				// Send 0 to stop reading.
	GPIO_Output(csPort, csPin, 1);  // Release rfid sensor
}

void RFID::RFID_SetRegisterBitMask(	uint8_t reg, uint8_t mask) {
	uint8_t tmp;
	tmp = RFID_ReadRegister(reg);
	RFID_WriteRegister(reg, tmp | mask);			// set bit mask
} // End RFID_SetRegisterBitMask()

void RFID::RFID_ClearRegisterBitMask(uint8_t reg, uint8_t mask){
	uint8_t tmp;
	tmp = RFID_ReadRegister(reg);
	RFID_WriteRegister(reg, tmp & (~mask));		// clear bit mask
}

RFID::StatusCode RFID::RFID_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result){
	uint16_t i;
	uint8_t n;

	RFID_WriteRegister(CommandReg, RFID_Idle);			// Stop any active command.
	RFID_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	RFID_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
//	RFID_WriteRegister(FIFODataReg, length, data);		// Write data to the FIFO
//	RFID_WriteRegister(CommandReg, RFID_CalcCRC);		// Start the calculation
	//Writing data to the FIFO
		for (i = 0; i < length; i++) {
			RFID_WriteRegister(FIFODataReg, *(data+i));
		}
		RFID_WriteRegister(CommandReg, RFID_CalcCRC);

	i = 0xFF;
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73�s.


	while (1) {
		n = RFID_ReadRegister(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		if (n & 0x04) {						// CRCIRq bit set - calculation done
			break;
		}
		if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the RFID might be down.
			return STATUS_TIMEOUT;
		}
	}
	RFID_WriteRegister(CommandReg, RFID_Idle);		// Stop calculating CRC for new content in the FIFO.
	
	// Transfer the result from the registers to the result buffer
	result[0] = RFID_ReadRegister(CRCResultRegL);
	result[1] = RFID_ReadRegister(CRCResultRegH);
	return STATUS_OK;
}

void RFID::RFID_Init(){
	setID(1);
	//MX_GPIO_Init();
	GPIO_Output(csPort, csPin, 0); //select rfid sensor

	RFID_Reset(); //soft reset
	// Reset baud rates
	RFID_WriteRegister(TxModeReg, 0x00);
	RFID_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	RFID_WriteRegister(ModWidthReg, 0x26);
	// When communicating with a RFID we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	RFID_WriteRegister(TModeReg, 0x80);	// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	RFID_WriteRegister(TPrescalerReg, 0xA9);// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25us.
	RFID_WriteRegister(TReloadRegH, 0x07);//  0x03 Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	RFID_WriteRegister(TReloadRegL, 0xD0); //0xE8


	//RFID_WriteRegister(RFCfgReg, 0X70); //48dB GAIN

	RFID_WriteRegister(TxASKReg, 0x40);	// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	RFID_WriteRegister(ModeReg, 0x3D);// 0x3D  Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	RFID_AntennaOn();
}

void RFID::RFID_Init(GPIO_TypeDef* _csPort, uint16_t _csPin){
    csPort = _csPort;
    csPin = _csPin;
	RFID_Init();
}


void RFID::RFID_Reset() {
	RFID_WriteRegister(CommandReg, RFID_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the RFID might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
	uint8_t count = 0;
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		HAL_Delay(50);
	} while ((RFID_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);



	//	HAL_Delay(50);
//	// Wait for the PowerDown bit in CommandReg to be cleared
//	while (RFID_ReadRegister(CommandReg) & (1<<4)) {
//		// RFID still restarting - unlikely after waiting 50ms, but better safe than sorry.
//	}
} // End RFID_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void RFID::RFID_AntennaOn() {
	uint8_t value = RFID_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		RFID_WriteRegister(TxControlReg, value | 0x03);
	}
} // End RFID_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void RFID::RFID_AntennaOff() {
	RFID_ClearRegisterBitMask(TxControlReg, 0x03);
} // End RFID_AntennaOff()

/**
 * Get the current RFID Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/RFID.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 *
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
uint8_t RFID::RFID_GetAntennaGain() {
	return RFID_ReadRegister(RFCfgReg) & (0x07<<4);
} // End RFID_GetAntennaGain()

/**
 * Set the RFID Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/RFID.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void RFID::RFID_SetAntennaGain(uint8_t mask) {
	if (RFID_GetAntennaGain() != mask) {						// only bother if there is a change
		RFID_ClearRegisterBitMask(RFCfgReg, (0x07<<4));		// clear needed to allow 000 pattern
		RFID_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4));	// only set RxGain[2:0] bits
	}
} // End RFID_SetAntennaGain()
RFID::StatusCode RFID::RFID_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
													uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
													uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
													uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
													uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default NULL.
													uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													bool checkCRC		///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
								 ) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return RFID_CommunicateWithPICC(RFID_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End RFID_TransceiveData()

/**
 * Transfers data to the RFID FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID::StatusCode RFID::RFID_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the RFID_Command enums.
														uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
														uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
														uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
														uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
														uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
														uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
														uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														bool checkCRC		///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
									 ) {
	uint8_t n, _validBits;
	unsigned int i;

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	RFID_WriteRegister(CommandReg, RFID_Idle);			// Stop any active command.
	RFID_WriteRegister(ComIrqReg, 0x7F);					//0X7F Clear all seven interrupt request bits
	RFID_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	RFID_WriteRegister(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	RFID_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	RFID_WriteRegister(CommandReg, command);				// Execute the command

	if (command == RFID_Transceive) {
		RFID_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In RFID_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the RFID stops transmitting.
	// Each iteration of the do-while-loop takes 17.86�s.
	i = 2000;
	while (1) {
		n = RFID_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
		if (--i == 0) {						// The emergency break. If all other conditions fail we will eventually terminate on this one after 35.7ms. Communication with the RFID might be down.
			return STATUS_TIMEOUT;
		}
	}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = RFID_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	// If the caller wants data back, get it from the RFID.
	if (backData && backLen) {
		n = RFID_ReadRegister(FIFOLevelReg);			// Number of uint8_ts in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of uint8_ts returned
		RFID_ReadRegister(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = RFID_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		RFID::StatusCode status = RFID_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}
	return STATUS_OK;
} // End RFID_CommunicateWithRFID()

/**
 * Transmits a REQuest command, Type A. Invites RFIDs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two RFIDs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID::StatusCode RFID::RFID_RequestA(uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
										) {
	return RFID_REQA_or_WUPA(RFID_CMD_REQA, bufferATQA, bufferSize);
} // End RFID_RequestA()


RFID::StatusCode RFID::RFID_WakeupA(uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
										) {
	return RFID_REQA_or_WUPA(RFID_CMD_WUPA, bufferATQA, bufferSize);
} // End RFID_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two RFIDs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID::StatusCode RFID::RFID_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize) {
	uint8_t validBits;
	RFID::StatusCode status;

	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 uint8_ts long.
		return STATUS_NO_ROOM;
	}
	RFID_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
	status = RFID_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End RFID_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single RFID.
 * Before calling this function the RFIDs must be placed in the READY(*) state by calling RFID_RequestA() or RFID_WakeupA().
 * On success:
 * 		- The chosen RFID is in state ACTIVE(*) and all other RFIDs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen RFID is returned in *uid along with the SAK.
 *
 * A RFID UID consists of 4, 7 or 10 uint8_ts.
 * Only 4 uint8_ts can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID uint8_ts		Cascade levels		Example of RFID
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID::StatusCode RFID::RFID_Select(Uid *uid, uint8_t validBits) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	RFID::StatusCode result;
	uint8_t count;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uiduint8_t[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 uint8_ts CRC_A
	uint8_t bufferUsed;				// The number of uint8_ts used in the buffer, ie the number of uint8_ts to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted uint8_t.
	uint8_t *responseBuffer;
	uint8_t responseLength;
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}

	// Prepare RFID
	RFID_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL uint8_t, find out if we need to use the Cascade Tag in uint8_t 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = RFID_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 uint8_ts
				break;

			case 2:
				buffer[0] = RFID_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 uint8_ts
				break;

			case 3:
				buffer[0] = RFID_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;

			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uiduint8_t[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = RFID_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of uint8_ts needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 uint8_ts in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and uint8_ts to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole uint8_ts
				// Calculate BCC - Block Check uint8_tacter
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = RFID_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 uint8_ts of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole uint8_ts in the UID part.
				index			= 2 + count;					// Number of whole uint8_ts: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			RFID_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = RFID_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
			if (result == STATUS_COLLISION) { // More than one RFID in the field => collision.
				uint8_t valueOfCollReg = RFID_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the RFID with the bit set.
				currentLevelKnownBits = collisionPos;
				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First uint8_t is index 0.
				buffer[index]	|= (1 << count);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID uint8_ts from buffer[] to uid->uiduint8_t[]
		index			= (buffer[2] == RFID_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == RFID_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 uint8_t + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those uint8_ts are not needed anymore.
		result = RFID_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End RFID_Select()

/**
 * Instructs a RFID in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.isNewCardPresent
 */
RFID::StatusCode RFID::RFID_HaltA() {
	RFID::StatusCode result;
	uint8_t buffer[4];

	// Build command buffer
	buffer[0] = RFID_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = RFID_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Send the command.
	// The standard says:
	//		If the RFID responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = RFID_TransceiveData(buffer, sizeof(buffer), NULL, 0);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End RFID_HaltA()



bool RFID::isNewCardPresent(){
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);

	// Reset baud rates
	RFID_WriteRegister(TxModeReg, 0x00);
	RFID_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	RFID_WriteRegister(ModWidthReg, 0x26);

	RFID::StatusCode result = RFID_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
}
bool RFID::readCardSerial(){
	RFID::StatusCode result = RFID_Select(&uid);
	return (result == STATUS_OK);
}

uint32_t RFID::readSensor(){
	// Wake PIC up
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
//	RFID_WriteRegister(TxModeReg, 0x00);
//	RFID_WriteRegister(RxModeReg, 0x00);
//	RFID_WriteRegister(ModWidthReg, 0x26);

	if(readCardSerial()) {
		printf("Tag ID: %02x%02x%02x%02x\r\n", uid.uidByte[0], uid.uidByte[1], uid.uidByte[2], uid.uidByte[3]);
	}
	else printf("Card Not Present\n\r");
	// Check if a card is present
	uint8_t result = RFID_WakeupA(bufferATQA, &bufferSize);
	if (result != STATUS_TIMEOUT)
		printf("read_card PICC_WakeupA: %d\r\n", result);
	if (result != STATUS_OK && result != STATUS_COLLISION)
		printf("read_card PICC_WakeupA: %d\r\n", result);
	// Read card
	result = RFID_Select(&uid);
	printf("Result RFID_Select: %d\r\n", result);
	if (result != STATUS_OK)
		return result;

	// Reset PIC
	do {
		result = RFID_HaltA();
		printf("reset PIC: %d\r\n", result);
	} while (result != STATUS_OK);

	return *((uint32_t*) &uid.uidByte);
}


int RFID::getSensorID(){
	return getID();
}





