/*

Module:  catena4802_rs485_demo.ino

Function:
        RS485 program for Catena 4802.

Copyright notice:
        This file copyright (C) 2023 by

                MCCI Corporation
                3520 Krums Corners Road
                Ithaca, NY  14850

        See project LICENSE file for license information.

Author:
        Pranau R, MCCI Corporation  June 2023

*/

#include <Catena.h>

#include <Catena_Led.h>
#include <Catena_TxBuffer.h>
#include <Catena_CommandStream.h>
#include <Catena_Mx25v8035f.h>

#include <Wire.h>
#include <hal/hal.h>
#include <mcciadk_baselib.h>

#include <cmath>
#include <type_traits>
#include <stm32_eeprom.h>
#include <Timer.h>

using namespace McciCatena;

/****************************************************************************\
|
|   Manifest Constants & Typedefs
|
\****************************************************************************/

/* adjustable timing parameters */
enum    {
        // set this to interval between transmissions, in seconds
        // Actual time will be a little longer because have to
        // add measurement and broadcast time, but we attempt
        // to compensate for the gross effects below.
        CATCFG_T_CYCLE = 6 * 60,        // every 6 minutes
        CATCFG_T_CYCLE_TEST = 30,       // every 30 seconds
        CATCFG_T_CYCLE_INITIAL = 30,    // every 30 seconds initially
        CATCFG_INTERVAL_COUNT_INITIAL = 10,     // repeat for 5 minutes
        CATCFG_T_REBOOT = 30 * 24 * 60 * 60,    // reboot every 30 days
        };

/* additional timing parameters; ususually you don't change these. */
enum    {
        CATCFG_T_WARMUP = 1,
        CATCFG_T_SETTLE = 5,
        CATCFG_T_OVERHEAD = (CATCFG_T_WARMUP + CATCFG_T_SETTLE + 4),
        CATCFG_T_MIN = CATCFG_T_OVERHEAD,
        CATCFG_T_MAX = CATCFG_T_CYCLE < 60 * 60 ? 60 * 60 : CATCFG_T_CYCLE,     // normally one hour max.
        CATCFG_INTERVAL_COUNT = 30,
        };

constexpr uint32_t CATCFG_GetInterval(uint32_t tCycle)
        {
        return (tCycle < CATCFG_T_OVERHEAD + 1)
                ? 1
                : tCycle - CATCFG_T_OVERHEAD
                ;
        }

enum    {
        CATCFG_T_INTERVAL = CATCFG_GetInterval(CATCFG_T_CYCLE),
        };

/****************************************************************************\
|
|   handy constexpr to extract the base name of a file
|
\****************************************************************************/

// two-argument version: first arg is what to return if we don't find
// a directory separator in the second part.
static constexpr const char *filebasename(const char *s, const char *p)
    {
    return p[0] == '\0'                     ? s                            :
           (p[0] == '/' || p[0] == '\\')    ? filebasename(p + 1, p + 1)   :
                                              filebasename(s, p + 1)       ;
    }

static constexpr const char *filebasename(const char *s)
    {
    return filebasename(s, s);
    }

/****************************************************************************\
|
|   Read-only data
|
\****************************************************************************/

static const char sVersion[] = "1.0.0";

/****************************************************************************\
|
|   VARIABLES
|
\****************************************************************************/

// the primary object
Catena gCatena;

//
// the LED
//
StatusLed gLed (Catena::PIN_STATUS_LED);

SPIClass gSPI2(
        Catena::PIN_SPI2_MOSI,
        Catena::PIN_SPI2_MISO,
        Catena::PIN_SPI2_SCK
        );

//  The flash
Catena_Mx25v8035f gFlash;
bool fFlash;

constexpr uint8_t channel = 4;
uint8_t nChannel;
uint8_t writeData[12];
uint8_t nData;
uint32_t timeOut = 3000;
uint32_t startTime;

#define UART2_BAUD_RATE     9600

#define UART2_RX_BUFFER_SIZE 64
#define UART2_TX_BUFFER_SIZE 64

#define UART2_RX_WAIT_TIME   3000   // ms

#define ASCII_STX     0x02    // Start of Text
#define ASCII_ACK     0x06    // Acknowledgment
#define ASCII_NAK     0x15    // Negative Acknowledgement
#define ASCII_DOLLOR  0x24    // $
#define ASCII_LF      0x0A    // Line Feed
#define ASCII_CR      0x0D    // Carriage Return
#define ASCII_SP      0x20    // Space

#define UART_READY      0
#define UART_RECEIVE    1
#define UART_READ       2
#define UART_WRITE      3
#define UART_TRANSMIT   4

byte com2TrxFlag = 0;         // unsigned char 0 ~ 255
byte com2RxHead = 0;
byte com2RxTail = 0;
byte com2TxHead = 0;
byte com2TxTail = 0;

volatile int8_t com2RxData[UART2_RX_BUFFER_SIZE] = {0};                    // 문자값 저장용 데이터 자료형, 문자값 => ASCII 값 저장, String 처리 가능
volatile int8_t com2TxData[UART2_TX_BUFFER_SIZE] = {0};

unsigned long lastTimeCom2Rx = 0;

unsigned long u32wait;

static constexpr uint8_t kVout1Enable = D10;
static constexpr uint8_t kVout2Enable = D11;
static constexpr uint8_t kExtendedI2cEn = D34;
static constexpr uint8_t kRs485Rx = D5;
static constexpr uint8_t kRs485Tx = D12;

static inline void vout1PowerOn(void)
    {
    pinMode(kVout1Enable, OUTPUT);
    digitalWrite(kVout1Enable, HIGH);
    }

static inline void vout1PowerOff(void)
    {
    pinMode(kVout1Enable, INPUT);
    digitalWrite(kVout1Enable, LOW);
    }

static inline void vout2PowerOn(void)
    {
    pinMode(kVout2Enable, OUTPUT);
    digitalWrite(kVout2Enable, HIGH);
    }

static inline void vout2PowerOff(void)
    {
    pinMode(kVout2Enable, INPUT);
    digitalWrite(kVout2Enable, LOW);
    }

static inline void kRs485RxEn(void)
    {
    digitalWrite(kRs485Rx, LOW);
    digitalWrite(kRs485Tx, LOW);
    }

static inline void kRs485TxEn(void)
    {
    digitalWrite(kRs485Rx, HIGH);
    digitalWrite(kRs485Tx, HIGH);
    delay(100);
    }

/*

Name:   setup()

Function:
        Arduino setup function.

Definition:
        void setup(
            void
            );

Description:
        This function is called by the Arduino framework after
        basic framework has been initialized. We initialize the sensors
        that are present on the platform, set up the LoRaWAN connection,
        and (ultimately) return to the framework, which then calls loop()
        forever.

Returns:
        No explicit result.

*/

void setup(void)
    {
    vout1PowerOn();         // internal peripherals Power on.
    vout2PowerOn();         // external peripherals power on.
    gCatena.begin();

    setup_platform();
    setup_flash();
    setup_rs485_serial();	// set upRS485 Serial
    }

void setup_platform(void)
    {
#ifdef USBCON
    // if running unattended, don't wait for USB connect.
    if (! (gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)))
        {
        while (!Serial)
            /* wait for USB attach */
            yield();
        }
#endif

    gCatena.SafePrintf("\n");
    gCatena.SafePrintf("-------------------------------------------------------------------------------\n");
    gCatena.SafePrintf("This is %s V%s.\n", filebasename(__FILE__), sVersion);
            {
            }
    gCatena.SafePrintf("Enter 'help' for a list of commands.\n");
    gCatena.SafePrintf("(remember to select 'Line Ending: Newline' at the bottom of the monitor window.)\n");

    gCatena.SafePrintf("SYSCLK: %u MHz\n", unsigned(gCatena.GetSystemClockRate() / (1000 * 1000)));

#ifdef USBCON
    gCatena.SafePrintf("USB enabled\n");
#else
    gCatena.SafePrintf("USB disabled\n");
#endif

    Catena::UniqueID_string_t CpuIDstring;

    gCatena.SafePrintf(
        "CPU Unique ID: %s\n",
        gCatena.GetUniqueIDstring(&CpuIDstring)
        );

    gCatena.SafePrintf("--------------------------------------------------------------------------------\n");
    gCatena.SafePrintf("\n");

    // set up the LED
    gLed.begin();
    gCatena.registerObject(&gLed);
    gLed.Set(LedPattern::FastFlash);

    /* find the platform */
    const Catena::EUI64_buffer_t *pSysEUI = gCatena.GetSysEUI();

    uint32_t flags;
    const CATENA_PLATFORM * const pPlatform = gCatena.GetPlatform();

    if (pPlatform)
        {
        gCatena.SafePrintf("EUI64: ");
        for (unsigned i = 0; i < sizeof(pSysEUI->b); ++i)
            {
            gCatena.SafePrintf("%s%02x", i == 0 ? "" : "-", pSysEUI->b[i]);
            }
        gCatena.SafePrintf("\n");
        flags = gCatena.GetPlatformFlags();
        gCatena.SafePrintf(
            "Platform Flags:  %#010x\n",
            flags
            );
        gCatena.SafePrintf(
            "Operating Flags:  %#010x\n",
            gCatena.GetOperatingFlags()
            );
        }
    else
        {
        gCatena.SafePrintf("**** no platform, check provisioning ****\n");
        flags = 0;
        }
    }

/*

Name:   setup_rs485_serial()

Function:
    Set up the RS485-Serial we intend to use (app specific).

Definition:
    void setup_rs485_serial(
        void
        );

Description:
    This function only exists to make clear what has to be done for
    the actual application. This is the code that cannot be part of
    the generic gCatena.begin() function.

Returns:
    No explicit result.

*/

void setup_rs485_serial(void)
    {
    pinMode(kRs485Rx, OUTPUT);
    pinMode(kRs485Tx, OUTPUT);
    Serial2.begin(9600);	// baud-rate at 9600
    delay(500);
    kRs485TxEn();
    Serial2.println("Hello VWI World!!! - Serial #2");    // Some leading characters are broken
    delay(500);
    kRs485RxEn();

//    nChannel = 1;
    u32wait = millis() + UART2_RX_WAIT_TIME;
    com2TrxFlag = UART_WRITE;
    }

void setup_flash(void)
    {
    if (gFlash.begin(&gSPI2, Catena::PIN_SPI2_FLASH_SS))
        {
        fFlash = true;
        gFlash.powerDown();
        gCatena.SafePrintf("FLASH found, put power down\n");
        }
    else
        {
        fFlash = false;
        gFlash.end();
        gSPI2.end();
        gCatena.SafePrintf("No FLASH found: check hardware\n");
        }
    }

uint32_t gRebootMs;

void loop()
    {
    gCatena.poll();
//  while (nChannel <= channel)
//      {
        if (com2TrxFlag == UART_WRITE)
            {
            // Serial.println(com2TrxFlag);
            com2TxDataLoad();
            }

        if (com2TrxFlag == UART_TRANSMIT)
            {
            // Serial.println(com2TrxFlag);
            kRs485TxEn();
            com2TxProcess();
//          nChannel = nChannel + 1;
//          if (nChannel > channel)
//              {
//              nChannel = 1;
//              }
            u32wait = millis() + 1000;         // wait a moment for tx data transmit
            }

        if (com2TrxFlag == UART_READY)
            {
            // Serial.println(com2TrxFlag);
            if (millis() > u32wait)
                {
                kRs485RxEn();
                com2TrxFlag = UART_RECEIVE;
                u32wait = millis() + UART2_RX_WAIT_TIME;      // wait a moment for slave answer
                }
            }

        if (com2TrxFlag == UART_RECEIVE)
            {
            // Serial.println(com2TrxFlag);
            if (millis() > u32wait)
                {
                com2TrxFlag = UART_WRITE;
                }
            }
        startTime = millis();

        while(!(nData = Serial2.available()))
            {
            if ((millis() - startTime) > timeOut)
                break;
            }

        if (nData > 0)
            {
            uint8_t *readData;
            gCatena.SafePrintf("nData: %u\n", nData);
//                        gCatena.SafePrintf("Channel %u Data:", nChannel);
            gCatena.SafePrintf("Channel Data:");
            for (uint8_t i = 0; i < nData; i++)
                {
                readData[i] = Serial2.read();
                gCatena.SafePrintf(" %02x", readData[i]);
                }
            gCatena.SafePrintf("\n");
            }
        else
            gCatena.SafePrintf("No data received from RS485 device\n");
//    }
    }

/******************************************************************************
|
|               RS485 Receive and Transmit functions
|
******************************************************************************/

bool com2RxDataLoad()
    {
    bool fResult = false;

    if ((com2TrxFlag == UART_READY) && Serial2.available())
        {
        volatile char udr2 = Serial2.read();
        if (udr2 == 0x30)
            {
            com2RxTail = 0;
            com2TrxFlag = UART_RECEIVE;
            }
        fResult = true;
        }

    if ((com2TrxFlag == UART_RECEIVE) && Serial2.available())
        {
        volatile char udr2 = Serial2.read();

        if ((udr2 == ASCII_CR) && (com2TrxFlag == UART_RECEIVE))
            {
            com2RxData[com2RxTail] = 0;
            }
        else if ((udr2 == ASCII_LF) && (com2TrxFlag == UART_RECEIVE))
            {
            com2TrxFlag = UART_READ;
            }
        else
            {
            if (com2TrxFlag == UART_RECEIVE)
                {
                com2RxData[com2RxTail++] = udr2;
                if (com2RxTail > UART2_RX_BUFFER_SIZE)
                        com2RxTail = UART2_RX_BUFFER_SIZE;
                }
            }
        fResult = true;
        }

    return fResult;
    }

void com2TxDataLoad()
{

  com2TxData[0] = '$';
  com2TxData[1] = 'S';
  com2TxData[2] = 'T';
  com2TxData[3] = 'N';
  com2TxData[4] = '_';
  com2TxData[5] = 'V';
  com2TxData[6] = 'W';
  com2TxData[7] = 'I';
  com2TxData[8] = ',';
  com2TxData[9] = '1';
  com2TxData[10] = '\r';
  com2TxData[11] = '\n';

  com2TrxFlag = UART_TRANSMIT;

}

void com2TxDataLoad(uint8_t nChannel)
    {
    com2TxData[0] = '$';
    com2TxData[1] = 'S';
    com2TxData[2] = 'T';
    com2TxData[3] = 'N';
    com2TxData[4] = '_';
    com2TxData[5] = 'V';
    com2TxData[6] = 'W';
    com2TxData[7] = 'I';
    com2TxData[8] = ',';
    com2TxData[9] = '0' + nChannel;
    com2TxData[10] = ASCII_CR;
    com2TxData[11] = ASCII_LF;

    com2TrxFlag = UART_TRANSMIT;
    }

void com2TxProcess()
    {
    Serial2.write(com2TxData[0]);
    Serial2.write(com2TxData[1]);
    Serial2.write(com2TxData[2]);
    Serial2.write(com2TxData[3]);
    Serial2.write(com2TxData[4]);
    Serial2.write(com2TxData[5]);
    Serial2.write(com2TxData[6]);
    Serial2.write(com2TxData[7]);
    Serial2.write(com2TxData[8]);
    Serial2.write(com2TxData[9]);     // nCh
    Serial2.write(com2TxData[10]);    // 0x0D
    Serial2.write(com2TxData[11]);    // 0x0A

    com2TrxFlag = UART_READY;
    }
