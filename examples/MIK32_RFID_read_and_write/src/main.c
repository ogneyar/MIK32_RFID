
#include "main.h"

MIFARE_Key key;         // Объект ключа
eStatusCode_t status; // Объект статуса


int main(void)
{
	InitClock();
    UART_Init(UART_0, (32000000/9600), UART_CONTROL1_TE_M | UART_CONTROL1_M_8BIT_M, 0, 0);
    xprintf("\r\nRFID read and write test run\r\n");

    SPI_Master_Init(); // Инициализация SPI
    
    RFID_init(SS_PORT, SS_PIN, RST_PORT, RST_PIN);

    for (byte i = 0; i < 6; i++) {  // Наполняем ключ
        // key.keyByte[i] = 0xFF;      // Ключ по умолчанию 0xFFFFFFFFFFFF
        key.keyByte[i] = 0xAB;      // 0xABABABABABAB
    }

    xprintf("Using key (for A and B):");
    dump_byte_array(key.keyByte, MF_KEY_SIZE);
    xprintf("\r\n");

    xprintf("BEWARE: Data will be written to the PICC, in sector #1\r\n");

    Uid my_uid;
    
    while(1)
    {
        // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
        if ( ! PICC_IsNewCardPresent() )
            continue;

        // Select one of the cards
        if ( ! PICC_ReadCardSerial() )
            continue;
        
        getUid(&my_uid);

        // Show some details of the PICC (that is: the tag/card)
        xprintf("\r\nCard UID:");
        dump_byte_array(my_uid.uidByte, my_uid.size);
        xprintf("\r\nPICC type: ");
        ePICC_Type_t piccType = PICC_GetType(my_uid.sak);
        xprintf(PICC_GetTypeName(piccType));
        xprintf("\r\n");

        // Check for compatibility
        if (    piccType != PICC_TYPE_MIFARE_MINI
            &&  piccType != PICC_TYPE_MIFARE_1K
            &&  piccType != PICC_TYPE_MIFARE_4K) {
            xprintf("\r\nThis sample only works with MIFARE Classic cards!!!\r\n");
            delay(500);
            continue;
        }

        // In this sample we use the second sector,
        // that is: sector #1, covering block #4 up to and including block #7
        byte sector         = 1;
        byte blockAddr      = 4;
        byte dataBlock[]    = {
            0x01, 0x02, 0x03, 0x04, //  1,  2,   3,  4,
            0x05, 0x06, 0x07, 0x08, //  5,  6,   7,  8,
            0x09, 0x0a, 0xff, 0x0b, //  9, 10, 255, 11,
            0x0c, 0x0d, 0x0e, 0x0f  // 12, 13, 14, 15
        };
        byte trailerBlock   = 7;
        byte buffer[18];
        byte size = sizeof(buffer);

        // Authenticate using key A
        xprintf("Authenticating using key A...");
        status = (eStatusCode_t) PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(my_uid));
        if (status != STATUS_OK) {
            xprintf("PCD_Authenticate() failed: ");
            xprintf(GetStatusCodeName(status));
            delay(500);
            continue;
        }

        // Show the whole sector as it currently is
        xprintf("Current data in sector:\r\n");
        PICC_DumpMifareClassicSectorToSerial(&(my_uid), &key, sector);
        xprintf("\r\n");

        // Read data from the block
        xprintf("Reading data from block "); 
        xprintf("%d", blockAddr);
        xprintf(" ...\r\n");
        status = (eStatusCode_t) MIFARE_Read(blockAddr, buffer, &size);
        if (status != STATUS_OK) {
            xprintf("MIFARE_Read() failed: ");
            xprintf(GetStatusCodeName(status));
            xprintf("\r\n");
        }
        xprintf("Data in block "); 
        xprintf("%d", blockAddr); 
        xprintf(":\r\n");
        dump_byte_array(buffer, 16); 
        xprintf("\r\n\r\n");

        // Authenticate using key B
        xprintf("Authenticating again using key B...\r\n");
        status = (eStatusCode_t) PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_B, trailerBlock, &key, &(my_uid));
        if (status != STATUS_OK) {
            xprintf("PCD_Authenticate() failed: ");
            xprintf(GetStatusCodeName(status));
            xprintf("\r\n");
            delay(500);
            continue;
        }

        // Write data to the block
        xprintf("Writing data into block "); 
        xprintf("%d", blockAddr);
        xprintf(" ...\r\n");
        dump_byte_array(dataBlock, 16); 
        xprintf("\r\n");
        status = (eStatusCode_t) MIFARE_Write(blockAddr, dataBlock, 16);
        if (status != STATUS_OK) {
            xprintf("MIFARE_Write() failed: ");
            xprintf(GetStatusCodeName(status));
            xprintf("\r\n");
        }
        xprintf("\r\n");

        // Read data from the block (again, should now be what we have written)
        xprintf("Reading data from block "); 
        xprintf("%d", blockAddr);
        xprintf(" ...\r\n");
        status = (eStatusCode_t) MIFARE_Read(blockAddr, buffer, &size);
        if (status != STATUS_OK) {
            xprintf("MIFARE_Read() failed: ");
            xprintf(GetStatusCodeName(status));
            xprintf("\r\n");
        }
        xprintf("Data in block "); 
        xprintf("%d", blockAddr); 
        xprintf(":\r\n");
        dump_byte_array(buffer, 16);
        xprintf("\r\n\r\n");

        // Check that data in block is what we have written
        // by counting the number of bytes that are equal
        xprintf("Checking result...\r\n");
        byte count = 0;
        for (byte i = 0; i < 16; i++) {
            // Compare buffer (= what we've read) with dataBlock (= what we've written)
            if (buffer[i] == dataBlock[i])
                count++;
        }
        xprintf("Number of bytes that match = "); 
        xprintf("%d", count);
        xprintf("\r\n");
        if (count == 16) {
            xprintf("Success :-)\r\n");
        } else {
            xprintf("Failure, no match :-(\r\n");
            xprintf("  perhaps the write didn't work properly...\r\n");
        }
        xprintf("\r\n");

        // Dump the sector data
        xprintf("Current data in sector:\r\n");
        PICC_DumpMifareClassicSectorToSerial(&(my_uid), &key, sector);
        xprintf("\r\n");

        // Halt PICC
        PICC_HaltA();
        // Stop encryption on PCD
        PCD_StopCrypto1();
    }
}


// Вывод массива hex значений в консоль
void dump_byte_array(byte *buffer, byte bufferSize) {
    const char Hex[2];
    for (byte i = 0; i < bufferSize; i++) {
        xprintf(" ");
        giveHexFromByte(buffer[i], &Hex);
        xprintf(Hex);
    }
}
