#ifndef SX1280FORRP2040
#define SX1280FORRP2040

/* void sx1280Rp2040Setup( uint8_t sx1280Rp2040BusyPin,
                        uint8_t sx1280Rp2040ResetPin,
                        uint8_t sx1280Rp2040SpiNumber,
                        uint8_t sx1280Rp2040SpiSckPin,
                        uint8_t sx1280Rp2040SpiTxPin,
                        uint8_t sx1280Rp2040SpiRxPin,
                        uint8_t sx1280Rp2040ChipSelectPin ); */
/* --------------------------- Structs -------------------------------- */

/* Lora Structs */
struct sx1280MessageStorage{

    uint8_t message[ 256 ];
};

struct rp2040Sx1280Pinout{

    uint8_t busyPin; 
    uint8_t resetPin;
    uint8_t spiNumber;
    uint8_t spiSckPin;
    uint8_t spiTxPin;
    uint8_t spiRxPin;
    uint8_t chipSelectPin;
};

struct sx1280LoraParameters{

    /* Setup Parameters */
    uint8_t standbyMode;
    uint8_t packetType;           /* Setting sx1280 Packet Type, lora=0x01 */
    uint32_t rfFrequency;         /* Setting RF Frequency, lower 24 bits used, upper 8 not */
    uint8_t txBaseAddress;        /* 8 bit integer, start of 256 byte message buffer for tx */
    uint8_t rxBaseAddress;        /* 8 bit integer, start of 256 byte message buffer for rx */
    uint8_t spreadingFactor;      /* Setting the Spreading Factor          */
    uint8_t bandwidth;            /* Setting the Bandwidth                 */
    uint8_t codingRate;           /* Setting the Coding Rate               */
    uint8_t preambleLength;
    uint8_t headerType;
    uint8_t cyclicalRedundancyCheck;
    uint8_t chirpInvert;
    uint8_t dioIrqParams;

    /* Tx Parameters */
    uint8_t power;
    uint8_t rampTime;
    uint16_t txIrq;                /* IRQ bitmask */
    uint8_t txPeriodBase;
    uint16_t txPeriodBaseCount;    /* Setting periodBaseCount */

    /* Rx Parameters */
    uint16_t rxIrq;                /* IRQ Mask for bits 15 to 8 of IRQ register */
    uint8_t rxPeriodBase;
    uint16_t rxPeriodBaseCount;    /* perdiodBase bits 15 to 8 for rx */
};

/* -------------------------------------------------------------------- */

void sx1280Rp2040Setup( struct rp2040sx1280Pinout thisRp2040sx1280Pinout );


static inline void sx1280Select( uint8_t sx1280ChipSelectPin );

static inline void sx1280Deselect( uint8_t sx1280ChipDeselectPin );

void getStatus( uint8_t statusChipSelectPin,  /* Passing chosen chip select pin       */
                uint8_t statusBusyPin,        /* Passing chosen busy pin for sx1280   */
                uint8_t statusSpiNumber );

/* Function sending common transciever settings to sx1280 */
void sx1280LoraSetup( struct sx1280LoraParameters setupSx1280LoraParameters, 
                      struct rp2040sx1280Pinout setupRp2040sx1280Pinout, 
                      uint8_t outboundMessage[ ] );
//void sx1280LoraSetup( uint8_t standbyMode,
//                  uint8_t packetType,           /* Setting sx1280 Packet Type, lora=0x01 */ 
//                  uint8_t rfFrequency2316,      /* Setting RF Frequency bits 23 to 16    */
//                  uint8_t rfFrequency158,       /* Setting RF Frequency bits 15 to 8     */
//                  uint8_t rfFrequency70,        /* Setting RF Frequency bits 7 to 0      */
//                  uint8_t spreadingFactor,      /* Setting the Spreading Factor          */
//                  uint8_t bandwidth,            /* Setting the Bandwidth                 */
//                  uint8_t codingRate,           /* Setting the Coding Rate               */
//                  uint8_t preambleLength, 
//                  uint8_t headerType, 
//                  uint8_t cyclicalRedundancyCheck,   
//                  uint8_t chirpInvert, 
//                  uint8_t setupChipSelectPin,   /* Passing chosen chip select pin        */
//                  uint8_t setupResetPin,        /* Passing chosen reset pin for sx1280   */
//                  uint8_t setupBusyPin,          /* Passing chosen busy pin for sx1280    */
//                  uint8_t setupSpiNumber,       /* Passing chosen SPI bus number         */
//                  uint8_t *outboundMessage );   /* Used to check length of message       */

/* Function setting up and running tx operation on an sx1280, taking 255 byte message packets */
uint64_t sx1280LoraTx( struct sx1280LoraParameters txSx1280LoraParameters, 
                       struct rp2040sx1280Pinout txRp2040sx1280Pinout, 
                       uint8_t outboundMessage[ ] );
//void sx1280LoraTx( uint8_t power, 
//               uint8_t rampTime,
//               uint8_t *outboundMessage,
//               uint8_t txIrq158,                /* IRQ Mask for bits 15:8 of IRQ register */
//               uint8_t txIrq70,                 /* IRQ Mask for bits 7:0 of IRQ register */
//               uint8_t txPeriodBase,
//               uint8_t txPeriodBaseCount158,    /* setting periodBaseCount bits 15 to 8 */
//               uint8_t txPeriodBaseCount70,     /* setting periodBaseCount bits 8 to 0 */
//               uint8_t txChipSelectPin,
//               uint8_t txBusyPin,
//uint8_t txSpiNumber );

/* Function setting up and running rx operation on an sx1280, 2.4Ghz LORA Modem*/
void sx1280LoraRx( struct sx1280LoraParameters rxSx1280LoraParameters, 
                   struct rp2040sx1280Pinout rxRp2040sx1280Pinout, 
                   uint8_t inboundMessage[ ] );
// void sx1280LoraRx( uint8_t rxIrq158,         /* IRQ Mask for bits 15 to 8 of IRQ register */
//               uint8_t rxIrq70,               /* IRQ Mask for bits 7 to 0 of IRQ register */
//               uint8_t rxPeriodBase,
//               uint8_t rxPeriodBaseCount158,  /* perdiodBase bits 15 to 8 for rx */
//               uint8_t rxPeriodBaseCount70,   /* perdiodBase bits 7 to 0 for rx */
//               uint8_t rxChipSelectPin,
//               uint8_t rxBusyPin,
//               uint8_t rxSpiNumber,
//               uint8_t *inboundMessage );     /* Pointer to return received message */
#endif
