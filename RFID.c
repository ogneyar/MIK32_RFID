
#include "RFID.h"
// #include "mik32_hal_scr1_timer.h"
#include <stddef.h>
#include <stdbool.h>


Uid uid;								// Used by PICC_ReadCardSerial().
uint16_t _chipSelectPin;
GPIO_TypeDef * _chipSelectPort;
uint16_t _resetPowerDownPin;
GPIO_TypeDef * _resetPowerDownPort;


void getUid(Uid * uuid)
{
	*uuid = uid;
}

// Возврат байта идентификатора
uint8_t get_uid(uint8_t number)
{
	return uid.uidByte[number];
}

// Инициализация таймера ядра scr1
static void Scr1_Timer_Init(void)
{
	HAL_Time_SCR1TIM_Init();
}

// Задержка в мс
void delay(uint32_t time)
{
	HAL_Time_SCR1TIM_DelayMs(time);
}

// Установка режима работы заданного пина
void pinMode(GPIO_TypeDef *Port, uint32_t Pin, uint8_t OutInput)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = Pin;
	GPIO_InitStruct.DS = HAL_GPIO_DS_2MA;

	if (OutInput) {
		// Port->DIRECTION_OUT = (Pin);
		GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
		GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
	}else {
		// Port->DIRECTION_IN = (Pin);
		GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_INPUT;
		GPIO_InitStruct.Pull = HAL_GPIO_PULL_UP;
	}
	HAL_GPIO_Init(Port, &GPIO_InitStruct);
}

// Чтение состояния заданного пина
uint8_t digitalRead(GPIO_TypeDef *Port, uint32_t Pin)
{
	if (Port->STATE & Pin) return 1;	
	return 0;
}

// Установка состояния заданного пина
void digitalWrite(GPIO_TypeDef *Port, uint32_t Pin, uint8_t SetReset)
{
	HAL_GPIO_WritePin(Port, Pin, SetReset);
}

// Инициализация RFID
void RFID_init(GPIO_TypeDef *chipSelectPort, uint16_t chipSelectPin, GPIO_TypeDef *resetPowerDownPort, uint16_t resetPowerDownPin)
{
	_chipSelectPin = chipSelectPin;
	_chipSelectPort = chipSelectPort;
	_resetPowerDownPin = resetPowerDownPin;
	_resetPowerDownPort = resetPowerDownPort;

	SPI_Master_Init(); // Инициализация SPI

  	Scr1_Timer_Init(); // необходимо для функции delay	

    PCD_Init();                     // Инициализация модуля
    PCD_SetAntennaGain(RxGain_max); // Установка усиления антенны
    PCD_AntennaOff();               // Перезагружаем антенну
    PCD_AntennaOn();                // Включаем антенну
}


void PCD_WriteRegister(ePCD_Register_t reg, byte value ) 
{
	digitalWrite(_chipSelectPort, _chipSelectPin, LOW);		// Select slave
	SPI_SendData(reg);
	SPI_SendData(value);
	digitalWrite(_chipSelectPort, _chipSelectPin, HIGH);		// Release slave again
} 


void PCD_WriteRegister_Array(ePCD_Register_t reg, byte count, byte *values)
{
	digitalWrite(_chipSelectPort, _chipSelectPin, LOW);		// Select slave
	SPI_SendData(reg);
	for (byte index = 0; index < count; index++) {
		SPI_SendData(values[index]);
	}
	digitalWrite(_chipSelectPort, _chipSelectPin, HIGH);		// Release slave again
}


byte PCD_ReadRegister(ePCD_Register_t reg	)
{
	byte value;
	digitalWrite(_chipSelectPort, _chipSelectPin, LOW);			// Select slave
	SPI_SendData(0x80 | reg);
	value = SPI_ReceiveData();
	digitalWrite(_chipSelectPort, _chipSelectPin, HIGH);			// Release slave again
	return value;
}


void PCD_ReadRegister_Array(ePCD_Register_t reg, byte count, byte *values, byte rxAlign) {
	if (count == 0) {
		return;
	}
	byte address = 0x80 | reg;
	byte index = 0;

	digitalWrite(_chipSelectPort, _chipSelectPin, LOW);		// Select slave
	count--;
	SPI_transfer(address);
	if (rxAlign) {
		byte mask = (0xFF << rxAlign) & 0xFF;
		byte value = SPI_transfer(address);
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}
	while (index < count) {
		values[index] = SPI_transfer(address);
		index++;
	}
	values[index] = SPI_transfer(0);
	digitalWrite(_chipSelectPort, _chipSelectPin, HIGH);	// Release slave again
}


void PCD_SetRegisterBitMask(ePCD_Register_t reg, byte mask)
{ 
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
}


void PCD_ClearRegisterBitMask(ePCD_Register_t reg, byte mask)
{
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
}


eStatusCode_t PCD_CalculateCRC(byte *data, byte length, byte *result) {
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_WriteRegister(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister_Array(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
	
    // HAL_SCR1_Timer_Start(&hscr1_timer, 89);
	uint32_t millis = HAL_Time_SCR1TIM_Millis();
	bool completed = false;

	do {
		byte n = PCD_ReadRegister(DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			PCD_WriteRegister(CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			result[0] = PCD_ReadRegister(CRCResultRegL);
			result[1] = PCD_ReadRegister(CRCResultRegH);
			completed = true;
			break;
		}
	}
	while ( ( HAL_Time_SCR1TIM_Millis() - millis ) < 89);
	// while (HAL_SCR1_Timer_GetFlagCMP(&hscr1_timer) == 0);

    // HAL_SCR1_Timer_Disable(&hscr1_timer);

	if (completed)
		return STATUS_OK;

	return STATUS_TIMEOUT;
}



/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

void PCD_Init(void) {
	bool hardReset = false;

	pinMode(_chipSelectPort, _chipSelectPin, _OUTPUT);
	digitalWrite(_chipSelectPort, _chipSelectPin, HIGH);
	
	if (_resetPowerDownPin != UNUSED_PIN) {
		pinMode(_resetPowerDownPort, _resetPowerDownPin, _INPUT);
	
		if (digitalRead(_resetPowerDownPort, _resetPowerDownPin) == LOW) {	// The MFRC522 chip is in power down mode.

			pinMode(_resetPowerDownPort, _resetPowerDownPin, _OUTPUT);		// Now set the resetPowerDownPin as digital output.
			digitalWrite(_resetPowerDownPort, _resetPowerDownPin, LOW);		// Make sure we have a clean LOW state.
			delay(1);
			digitalWrite(_resetPowerDownPort, _resetPowerDownPin, HIGH);		// Exit power down mode. This triggers a hard reset.
			delay(50);
			hardReset = true;
		}
	}

	if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
		PCD_Reset();
	}
	
	// Reset baud rates
	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);

	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);
	
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}


void PCD_Reset(void) {
	PCD_WriteRegister(CommandReg, PCD_SoftReset);
	
	uint8_t count = 0;
	do {
		delay(50);
	} while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
}


void PCD_AntennaOn(void) {
	byte value = PCD_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
	}
}


void PCD_AntennaOff(void) {
	PCD_ClearRegisterBitMask(TxControlReg, 0x03);
}


byte PCD_GetAntennaGain(void) {
	return PCD_ReadRegister(RFCfgReg) & (0x07<<4);
}


void PCD_SetAntennaGain(byte mask) {
	if (PCD_GetAntennaGain() != mask) {						// only bother if there is a change
		PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));		// clear needed to allow 000 pattern
		PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4));	// only set RxGain[2:0] bits
	}
}



/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

eStatusCode_t PCD_TransceiveData(byte *sendData,		///< Pointer to the data to transfer to the FIFO.
								 byte sendLen,		///< Number of bytes to transfer to the FIFO.
								 byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
								 byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
								 byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
								 byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
								 byte checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
) {
	byte waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}


eStatusCode_t PCD_CommunicateWithPICC(byte command,		///< The command to execute. One of the PCD_Command enums.
									  byte waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
									  byte *sendData,		///< Pointer to the data to transfer to the FIFO.
									  byte sendLen,		///< Number of bytes to transfer to the FIFO.
									  byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
									  byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
									  byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
									  byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
									  byte checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
) {
	byte txLastBits = validBits ? *validBits : 0;
	byte bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_WriteRegister(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization

	PCD_WriteRegister_Array(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO

	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}	

    // HAL_SCR1_Timer_Start(&hscr1_timer, 36);
	uint32_t millis = HAL_Time_SCR1TIM_Millis();
	bool completed = false;

	do {
		byte n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			completed = true;
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			break;
		}
	}
	while ( ( HAL_Time_SCR1TIM_Millis() - millis ) < 36);
	// while (HAL_SCR1_Timer_GetFlagCMP(&hscr1_timer) == 0);

    // HAL_SCR1_Timer_Disable(&hscr1_timer);

	if (!completed) {
		return STATUS_TIMEOUT;
	}
	
	byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
  
	byte _validBits = 0;
	
	if (backData && backLen) {
		byte n = PCD_ReadRegister(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegister_Array(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}
	
	if (backData && backLen && checkCRC) {
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		byte controlBuffer[2];
		eStatusCode_t status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End PCD_CommunicateWithPICC()


eStatusCode_t PICC_RequestA(byte *bufferATQA, byte *bufferSize)
{
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
}


eStatusCode_t PICC_WakeupA(byte *bufferATQA, byte *bufferSize)
{
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
}


eStatusCode_t PICC_REQA_or_WUPA(byte command, byte *bufferATQA, byte *bufferSize)
{
	byte validBits;
	eStatusCode_t status;
	
	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
}


eStatusCode_t PICC_Select(Uid *uid, byte validBits)
{
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	byte cascadeLevel = 1;
	eStatusCode_t result;
	byte count;
	byte checkBit;
	byte index;
	byte uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	byte *responseBuffer;
	byte responseLength;
	
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	uidComplete = false;
	while (!uidComplete) {
		
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		selectDone = false;
		while (!selectDone) {
			
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				byte valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
				}
				else { // This was an ANTICOLLISION.
					currentLevelKnownBits = 32;
				}
			}
		} // End of while (!selectDone)
		
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
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
		
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()



eStatusCode_t PICC_HaltA(void) {
	eStatusCode_t result;
	byte buffer[4];
	
	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	
	result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0, NULL, 0, false);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()




/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

eStatusCode_t PCD_Authenticate(byte command, byte blockAddr, MIFARE_Key *key, Uid *uid)
{
	byte waitIRq = 0x10;		// IdleIRq
	
	byte sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (byte i = 0; i < MF_KEY_SIZE; i++) {	// 6 key bytes
		sendData[2+i] = key->keyByte[i];
	}
	
	for (byte i = 0; i < 4; i++) {				// The last 4 bytes of the UID
		sendData[8+i] = uid->uidByte[i+uid->size-4];
	}
	
	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData), NULL, NULL, NULL, 0, false);
} // End PCD_Authenticate()


void PCD_StopCrypto1(void) {
	PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()


eStatusCode_t MIFARE_Read(byte blockAddr, byte *buffer, byte *bufferSize)
{
	eStatusCode_t result;
	
	if (buffer == NULL || *bufferSize < 18) {
		return STATUS_NO_ROOM;
	}
	
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, true);
} // End MIFARE_Read()


eStatusCode_t MIFARE_Write(byte blockAddr, byte *buffer, byte bufferSize)
{
	eStatusCode_t result;
	
	if (buffer == NULL || bufferSize < 16) {
		return STATUS_INVALID;
	}
	
	byte cmdBuffer[2];
	cmdBuffer[0] = PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(cmdBuffer, 2, false); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	
	result = PCD_MIFARE_Transceive(buffer, bufferSize, false); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	
	return STATUS_OK;
} // End MIFARE_Write()




/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////

eStatusCode_t PCD_MIFARE_Transceive(byte *sendData,	byte sendLen, byte acceptTimeout)
{
	eStatusCode_t result;
	byte cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.
	
	if (sendData == NULL || sendLen > 16) {
		return STATUS_INVALID;
	}
	
	memcpy(cmdBuffer, sendData, sendLen);
	result = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
	if (result != STATUS_OK) { 
		return result;
	}
	sendLen += 2;
	
	byte waitIRq = 0x30;		// RxIRq and IdleIRq
	byte cmdBufferSize = sizeof(cmdBuffer);
	byte validBits = 0;
	result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits, 0, false);
	if (acceptTimeout && result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result != STATUS_OK) {
		return result;
	}
	// The PICC must reply with a 4 bit ACK
	if (cmdBufferSize != 1 || validBits != 4) {
		return STATUS_ERROR;
	}
	if (cmdBuffer[0] != MF_ACK) {
		return STATUS_MIFARE_NACK;
	}
	return STATUS_OK;
} // End PCD_MIFARE_Transceive()


const char * GetStatusCodeName(eStatusCode_t code)
{
	switch (code) {
		case STATUS_OK:				return "Success.";
		case STATUS_ERROR:			return "Error in communication.";
		case STATUS_COLLISION:		return "Collision detected.";
		case STATUS_TIMEOUT:		return "Timeout in communication.";
		case STATUS_NO_ROOM:		return "A buffer is not big enough.";
		case STATUS_INTERNAL_ERROR:	return "Internal error in the code. Should not happen.";
		case STATUS_INVALID:		return "Invalid argument.";
		case STATUS_CRC_WRONG:		return "The CRC_A does not match.";
		case STATUS_MIFARE_NACK:	return "A MIFARE PICC responded with NAK.";
		default:					return "Unknown error";
	}
} // End GetStatusCodeName()


ePICC_Type_t PICC_GetType(byte sak)
{
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
} // End PICC_GetType()


const char * PICC_GetTypeName(ePICC_Type_t piccType)
{
	switch (piccType) {
		case PICC_TYPE_ISO_14443_4:		return "PICC compliant with ISO/IEC 14443-4";
		case PICC_TYPE_ISO_18092:		return "PICC compliant with ISO/IEC 18092 (NFC)";
		case PICC_TYPE_MIFARE_MINI:		return "MIFARE Mini, 320 bytes";
		case PICC_TYPE_MIFARE_1K:		return "MIFARE 1KB";
		case PICC_TYPE_MIFARE_4K:		return "MIFARE 4KB";
		case PICC_TYPE_MIFARE_UL:		return "MIFARE Ultralight or Ultralight C";
		case PICC_TYPE_MIFARE_PLUS:		return "MIFARE Plus";
		case PICC_TYPE_MIFARE_DESFIRE:	return "MIFARE DESFire";
		case PICC_TYPE_TNP3XXX:			return "MIFARE TNP3XXX";
		case PICC_TYPE_NOT_COMPLETE:	return "SAK indicates UID is not complete.";
		case PICC_TYPE_UNKNOWN:
		default:						return "Unknown type";
	}
} // End PICC_GetTypeName()


void PICC_DumpMifareClassicSectorToSerial(Uid *uid, MIFARE_Key *key, byte sector)
{
	const char Hex[2];
	eStatusCode_t status;
	byte firstBlock;		// Address of lowest address to dump actually last block dumped)
	byte no_of_blocks;		// Number of blocks in sector
	bool isSectorTrailer;	// Set to true while handling the "last" (ie highest address) in the sector.
	
	byte c1, c2, c3;		// Nibbles
	byte c1_, c2_, c3_;		// Inverted nibbles
	bool invertedError;		// True if one of the inverted nibbles did not match
	byte g[4];				// Access bits for each of the four groups.
	byte group;				// 0-3 - active group for access bits
	bool firstInGroup;		// True for the first block dumped in the group
	
	if (sector < 32) { // Sectors 0..31 has 4 blocks each
		no_of_blocks = 4;
		firstBlock = sector * no_of_blocks;
	}
	else if (sector < 40) { // Sectors 32-39 has 16 blocks each
		no_of_blocks = 16;
		firstBlock = 128 + (sector - 32) * no_of_blocks;
	}
	else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
		return;
	}
		
	byte byteCount;
	byte buffer[18];
	byte blockAddr;
	isSectorTrailer = true;
	invertedError = false;	// Avoid "unused variable" warning.
	for (int8_t blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) {
		blockAddr = firstBlock + blockOffset;
		
		if (isSectorTrailer) {
			if(sector < 10)
				xprintf("   "); // Pad with spaces
			else
				xprintf("  "); // Pad with spaces
			xprintf(sector);
			xprintf("  ");
		}
		else {
			xprintf("       ");
		}
		// Block number
		if(blockAddr < 10)
			xprintf("   "); // Pad with spaces
		else {
			if(blockAddr < 100)
				xprintf("  "); // Pad with spaces
			else
				xprintf(" "); // Pad with spaces
		}
		xprintf("%d", blockAddr);
		xprintf("  ");
		// Establish encrypted communications before reading the first block
		if (isSectorTrailer) {
			status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
			if (status != STATUS_OK) {
				xprintf("PCD_Authenticate() failed: ");
				xprintf(GetStatusCodeName(status));
				xprintf("\r\n");
				return;
			}
		}
		// Read block
		byteCount = sizeof(buffer);
		status = MIFARE_Read(blockAddr, buffer, &byteCount);
		if (status != STATUS_OK) {
			xprintf("MIFARE_Read() failed: ");
			xprintf(GetStatusCodeName(status));
			xprintf("\r\n");
			continue;
		}
		
		for (byte index = 0; index < 16; index++) {
			// if(buffer[index] < 0x10)
			// 	xprintf(" 0");
			// else
				xprintf(" ");
			giveHexFromByte(buffer[index], &Hex);
			xprintf(Hex);
			if ((index % 4) == 3) {
				xprintf(" ");
			}
		}
		
		if (isSectorTrailer) {
			c1  = buffer[7] >> 4;
			c2  = buffer[8] & 0xF;
			c3  = buffer[8] >> 4;
			c1_ = buffer[6] & 0xF;
			c2_ = buffer[6] >> 4;
			c3_ = buffer[7] & 0xF;
			invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
			g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
			g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
			g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
			g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
			isSectorTrailer = false;
		}
		
		if (no_of_blocks == 4) {
			group = blockOffset;
			firstInGroup = true;
		}
		else {
			group = blockOffset / 5;
			firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
		}
		
		if (firstInGroup) {
			xprintf(" [ ");
			xprintf("%d", (g[group] >> 2) & 1); 
			xprintf(" ");
			xprintf("%d", (g[group] >> 1) & 1); 
			xprintf(" ");
			xprintf("%d", (g[group] >> 0) & 1); 
			xprintf(" ] ");
			if (invertedError) {
				xprintf(" Inverted access bits did not match! ");
			}
		}
		
		if (group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
			int32_t value = ((int32_t)(buffer[3])<<24) | ((int32_t)(buffer[2])<<16) | ((int32_t)(buffer[1])<<8) | (int32_t)(buffer[0]);
			xprintf(" Value=0x"); 
			giveHexFromByte(value, &Hex);
			xprintf(Hex);
			xprintf(" Adr=0x");
			giveHexFromByte(buffer[12], &Hex);
			xprintf(Hex);
		}
		xprintf("\r\n");
	}
	
	return;
} // End PICC_DumpMifareClassicSectorToSerial()



/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

bool PICC_IsNewCardPresent() {
	byte bufferATQA[2];
	byte bufferSize = sizeof(bufferATQA);

	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	PCD_WriteRegister(ModWidthReg, 0x26);

	enum StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);

	return (result == STATUS_OK || result == STATUS_COLLISION);
}


bool PICC_ReadCardSerial() {
	enum StatusCode result = PICC_Select(&uid, 0);
	return (result == STATUS_OK);
}

