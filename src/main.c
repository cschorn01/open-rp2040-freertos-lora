/*
 * Author: Chris Schorn
 * Open Lora Mesh Network
 * Version: 
 * Date:
 * Versioning Reason:
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* 
                General Structure Notes 

    In main, setup tasks and run vTaskStartScheduler.
    In tasks, setup hardware before while(true), and
        run data transfer with hardware in while(true).
    Mesh network processing will move to the second core
        to continue in the background if needed
    Use "back" input from touchscreen to put current
        task/app in the blocked state and go home screen
    Using task notifications, I can take the 'notification
        value' and convert it to an 32 bit pointer address
*/

/* FreeRTOS Includes */
#include "../FreeRTOS-Kernel/include/FreeRTOS.h" /* MUST COME FIRST */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */

/* Raspberry Pi Pico Inlcudes */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

/* Including my libraries */
#include "../myLibraries/sx1280ForRp2040.h"

static TaskHandle_t xSimpleLEDTaskHandle = NULL;
static TaskHandle_t xUsbIOTaskHandle = NULL;
static TaskHandle_t xSx1280TaskHandle = NULL;

/* --------------------------- Macros -------------------------------- */

/* Allows the spi instance to be variable based on a 1 or 0 input */
#define I2CSELECT(I2CNum)   (I2CNum == 0 ? i2c0 : (I2CNum == 1 ? i2c1 : NULL))
#define UARTSELECT(UARTNum) (UARTNum == 0 ? uart0 : (UARTNum == 1 ? uart1 : NULL))

/* -------------------------- Random Number Generator ----------------------------------- */

/* Generating random 32 bit integer from a floating analog pin
   Only generating integers within a certain ranger through bitmasking 
        Only able to do this because it's taking long enough and intervals are not needed at the moment
   Foating analogue pin will output random values because it is receiving noise from
        electronics on the board and electromagnetic interference from external sources */
uint32_t floatingAnalogRNG( uint32_t lowerBound, uint32_t upperBound, uint32_t interval ){

    uint16_t rawADC = 0;
    uint16_t rawADCStorage = 0;
    const float conversionFactor = 3.3f / (1 << 12 ); /* Unsure but its in Rpi pico docs */
    uint32_t randomNumberOutput = 0;
    uint32_t uint32_max = 0xFFFFFFFF;

    uint32_t i = 0;

    /* Initializing ADC for psuedo-random number generator off noise on pin */
    adc_init();
    adc_gpio_init( 0 );
    adc_set_temp_sensor_enabled( false );
    adc_select_input( 26 );
    adc_fifo_setup( true, false, 0, false, false );

    /* Loop generating a 32 bit random number
       Checks whether the current ADC value read in from pin 26, rawADC, is larger, 
            smaller or equal to the previous ADC value read in from pin 26, rawADCStorage.
       If rawADCStorage is less than rawADC the 32 bit number is bit shifted left once
       If rawADCStorage is greater than rawADC the 32 bit number is bit shifted left once,
            and a 1 will be added to the right most bit
       If rawADCStorage is equal to rawADC the 32 bit number is bit shifted left once, 
            and copy the second from the right most bit into the right most bit */
    for( i = 0; i < 40; i++ ){

        // vTaskDelay( 20 );

        rawADC = adc_read();
        /* printf("rawADC: %f\n", rawADC * conversionFactor ); */

        if( i != 0 && rawADCStorage < rawADC ){
            randomNumberOutput = ( randomNumberOutput << 1 );
        }
        else if( i != 0 && rawADCStorage > rawADC ){
            randomNumberOutput = ( randomNumberOutput << 1 ) | 0x00000001;
        }
        else if( i != 0 && rawADCStorage == rawADC ){// ( randomNumberOutput & 0x00000001 == 0x00000001 ) 
            if( randomNumberOutput & 0x00000001 == 0x00000001 ){
                randomNumberOutput = ( randomNumberOutput << 1 ) | 0x00000001;
            }
            else if( randomNumberOutput & 0x00000001 == 0x00000000 ){
                randomNumberOutput = ( randomNumberOutput << 1 );
            }
        }

        rawADCStorage = rawADC;
        /* printf("Random Number Loop: 0x%X %u\n", randomNumberOutput, randomNumberOutput ); */
    }

    /* Taking the ratio of random 32 bit number against the maximum number that can be
            generated, multiplying the result with the number of numbers allowed between 
            the bounds and interval, the result of which is multiplied by the interval to
            give how much higher than the lower bound the output number should be */
    randomNumberOutput = ( ( ( ( randomNumberOutput * 1.0f )/uint32_max ) * ( ( upperBound - lowerBound )/interval ) ) * interval ) + lowerBound;
    /* printf("Random Number Output: 0x%X %u\n", randomNumberOutput, randomNumberOutput ); */

    return randomNumberOutput;
}

/* --------------------------- sx1280 2.4GHz Lora Operation -------------------------------- */

/*  Task setting up and running sx1280
    Multi-packet messaging commented out, putting in single packet */
void vSx1280Task( void *pvParameters ){

    /* Iterators */
    uint32_t i = 0;
    uint32_t j = 0;

    /* Timer Variables */
    uint64_t currentTimeStamp = 0;
    uint64_t timeStampHolder = 0;
    uint64_t txTimeStamp = 0;
    uint64_t rxTimesStamp = 0;

    /* LoRa variables */
    uint16_t sx1280Irq = 0; /* 16 bit integer holding IRQ returned for Tx and Rx operation */
    uint16_t packetStatus = 0;
    uint16_t bufferSizeAndStart = 0; /* Payload length in high byte, buffer start in low byte */

    /* 32 bit pointer for address received from vUsbIOTask task notification */
    uint32_t *taskNotificationFromUSB = ( uint32_t * ) pvPortMalloc( 1*sizeof( uint32_t ) );

    /* Instantiating array of structs to store messages till broadcast or stored in SD Card */
    uint32_t messageStorageSize = 1;
    struct sx1280MessageStorage messageStorage[ messageStorageSize ];
    uint8_t messageStorageContext = 0; /* Bits show which arrays in messageStorage are empty */
    /* Loop is only here so I dont have to edit { 0 } style instantiation for each change */
    for( i = 0; i < messageStorageSize; i++ ) 
        memset( messageStorage[ i ].message, 0, 256*sizeof( uint8_t ) );

    /* 32 bits for analog pin random number generator output */
    uint32_t randomAnalogOutput = 0;

    struct sx1280LoraParameters deviceSx1280LoraParameters = defaultSx1280LoraParameters;

    /* Initalizing unsigned 8 bit integers to store pin assignments for the sx1280 */
    struct rp2040sx1280Pinout deviceRp2040sx1280Pinout = { 22,   /* sx1280BusyPin       */
                                                           21,   /* sx1280ResetPin      */
                                                           1,    /* sx1280SpiNumber     */
                                                           10,   /* sx1280SpiSckPin     */
                                                           11,   /* sx1280SpiTxPin      */
                                                           12,   /* sx1280SpiRxPin      */
                                                           13 }; /* sx1280ChipSelectPin */

    sx1280Rp2040Setup( deviceRp2040sx1280Pinout );

    // may switch antsel to GPIO20 because it's cleaner on pcb
    uint8_t antselPin = 28;         /* Antenna Select pin for DLP-RFS1280 module            */
    gpio_set_dir( antselPin, 1 );   /* Set GPIO to output, True = out False = in            */
    gpio_put( antselPin, 0 );       /* GPIO Low, using DLP-RFS1280 module onboard antenna   */

    rp2040Sx1280Reset( deviceRp2040sx1280Pinout );

    while( true ){

        vTaskDelay( 10 ); /* Allow other tasks to run */

        *( taskNotificationFromUSB ) = 0;
        xTaskNotifyWait( 0xffffffff,               /* uint32_t ulBitsToClearOnEntry */
                         0,                        /* uint32_t ulBitsToClearOnExit */
                         taskNotificationFromUSB,  /* uint32_t *pulNotificationValue */
                         100 );                    /* TickType_t xTicksToWait */
 
        /* Check task notification isn't empty */
        if( *( taskNotificationFromUSB ) != 0 && messageStorage[ 0 ].message[ 0 ] == 0 ){

            for( j = 0; *( ( uint8_t * ) *( taskNotificationFromUSB ) + j ) != 0x00; j++){
                printf("sx1280 task notification = 0x%X %c\n", *( ( uint8_t * ) *( taskNotificationFromUSB ) + j ), *( ( uint8_t * ) *( taskNotificationFromUSB ) + j ) );
            }
            // printf("sx1280 task notification held address: 0x%X\n", *(taskNotificationFromUSB));
            memcpy( messageStorage[ 0 ].message, ( uint8_t * ) *( taskNotificationFromUSB ), 256 );
        }

        /* ---------------------- rp2040 sx1280 Tx Operation -------------------------- */

        /* Check that we are ready to send any type of message
           Currently just that the storage array is not empty */
        if( messageStorage[ 0 ].message[ 0 ] != 0 ){

            /* ------------------ rp2040 sx1280 Tx Setup ------------------- */
            // CHANGE FUNCTIONS TO PASS STRUCTS BY REFERENCE, NOT VALUE SO NO STACK REPEATS

            rp2040Sx1280SetStandy( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            rp2040Sx1280setPacketType( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            rp2040Sx1280setRfFrequency( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            rp2040Sx1280setBufferBase( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            rp2040Sx1280setModulationParams( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout);

            rp2040Sx1280setLoraPacketParams( deviceSx1280LoraParameters,
                                             deviceRp2040sx1280Pinout,
                                             getPayloadLength( messageStorage[ 0 ].message ));

            /* rp2040Sx1280SetupTest( deviceRp2040sx1280Pinout ); uncomment to test setup */

            /* ------------------------- rp2040 sx1280 Tx Send --------------------------- */

            rp2040Sx1280setTxParams( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            rp2040Sx1280WriteBuffer( deviceSx1280LoraParameters, 
                                     deviceRp2040sx1280Pinout,
                                     getPayloadLength( messageStorage[ 0 ].message ),
                                     messageStorage[ 0 ].message );

            rp2040Sx1280SetIrqParams( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            /* sx1280 Doc says to do before SetTx */
            rp2040Sx1280ClearIrq( deviceRp2040sx1280Pinout ); 

            timeStampHolder = time_us_64( ); /* For LoraWAN operation */
            rp2040Sx1280SetTx( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            /* Loop for checking Tx was sent, timed out, breaks if Tx single mode runs long */
            for( i = 0; i < 50; i++ ){

                sx1280Irq = rp2040Sx1280getIrqStatus( deviceRp2040sx1280Pinout );

                printf("IRQ Check: 0x%X %i\n", sx1280Irq, i );

                /* Checking IRQ to see if the TxDone bit is high */
                if( sx1280Irq & 0x01 == 0x01 ){ 

                    /* Using 64 bit timestamp value in microseconds for LoRaWAN timing */
                    txTimeStamp = time_us_64( );
                    printf("IRQ: 0x%X %i \n", *( txReadData + 3 ), i );
                    /* Clear successfully sent message */
                    memset( messageStorage[ 0 ].message, 0, 256*sizeof( uint8_t ) );
                    break;
                }
                /* Checking IRQ to see if the rxTxTimeout bit is high */
                else if( sx1280Irq & 0x40 == 0x40 ){
                    
                    printf("IRQ: 0x%X %i \n", *( txReadData + 3 ), i );
                    break;
                }
               /* else if( sx1280 IRQ is no 0x02 or 0x40, message has issues in this case ){
                    break;
                } */

                vTaskDelay( 50 );
                currentTimeStamp = time_us_64( );
            }

            rp2040Sx1280ClearIrq( deviceRp2040sx1280Pinout );

            rp2040Sx1280SetStandy( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );
        }

        /* ---------------------- rp2040 sx1280 Rx Operation -------------------------- */

        /* Check that we are ready to receive any type of message
           Currently that the storage array is empty */
        if( messageStorage[ 0 ].message == 0x00 || isMessageLorawan( messageStorage[ 0 ].message, deviceLorawanParameters.deviceAddress ) ){

            /* Checking the message does not have LoRaWAN device address, so not LoRaWAN */
            if( !isMessageLorawan( messageStorage[ 0 ].message, deviceLorawanParameters.deviceAddress ) ){
                /* Setting txTimeStamp to 0, no RX delay for not LoraWAN operation */
                txTimeStamp = 0;
            }

            /* ------------------ rp2040 sx1280 Rx Setup ------------------- */

            rp2040Sx1280SetStandy( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            rp2040Sx1280setPacketType( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            rp2040Sx1280setRfFrequency( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            rp2040Sx1280setBufferBase( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            rp2040Sx1280setModulationParams( deviceSx1280LoraParameters,
                                             deviceRp2040sx1280Pinout );

            rp2040Sx1280setLoraPacketParams( deviceSx1280LoraParameters,
                                             deviceRp2040sx1280Pinout,
                                             getPayloadLength( messageStorage[ 0 ].message ));

            /* rp2040Sx1280SetupTest( deviceRp2040sx1280Pinout ); uncomment to test setup */

            /* ---------------------- rp2040 sx1280 Rx Listen -------------------------- */

            rp2040Sx1280SetIrqParams( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            /* sx1280 Doc says to do before SetTx */
            rp2040Sx1280ClearIrq( deviceRp2040sx1280Pinout ); 

            timeStampHolder = time_us_64( );
            rp2040Sx1280SetRx( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );

            /* Loop for checking Tx was sent, timed out, breaks if Tx single mode runs long */
            for( i = 0; i < 50; i++ ){

                sx1280Irq = rp2040Sx1280getIrqStatus( deviceRp2040sx1280Pinout );
                printf("IRQ Check: 0x%X %i\n", sx1280Irq, i );

                /* Checking IRQ to see if the RxDone, or rxTxTimeout bits are high */
                if( sx1280Irq & 0x01 == 0x01 ){ 

                    /* Using 64 bit timestamp value in microseconds for LoRaWAN timing */
                    rxTimeStamp = time_us_64( );
                    printf("Message Sent IRQ: 0x%X %i \n", sx1280Irq, i );

                    packetStatus = rp2040Sx1280LoraGetPacketStatus( deviceRp2040sx1280Pinout );
                    rp2040Sx1280ClearIrq( deviceRp2040sx1280Pinout );
                    /* Payload length in high byte, buffer start in low byte */
                    bufferSizeAndStart = rp2040Sx1280GetRxBufferStatus( deviceRp2040sx1280Pinout );
                    rp2040Sx1280ReadBuffer( deviceRp2040sx1280Pinout,
                                            bufferSizeAndStart, 
                                            messageStorage[ 0 ].message );
                    break;
                }
                else if( sx1280Irq & 0x40 == 0x40 ){

                    printf("Timeout IRQ: 0x%X %i \n", sx1280Irq, i );
                    break;
                }
                else{
                    printf("Rx IRQ: 0x%X%X", ( uint8_t ) sx1280Irq >> 8, (uint8_t ) sx1280Irq );
                }
               /* else if( sx1280 IRQ is no 0x02 or 0x40, message has issues in this case ){
                    break;
                } 
                if( deviceSx1280LoraParameters.rxPeriodBaseCount == 0x0000 || deviceSx1280LoraParameters.rxPeriodBaseCount == 0xFFFF ){ } */

                vTaskDelay( 50 );
                currentTimeStamp = time_us_64( );
            }

            rp2040Sx1280ClearIrq( deviceRp2040sx1280Pinout );

            rp2040Sx1280SetStandy( deviceSx1280LoraParameters, deviceRp2040sx1280Pinout );
        }
    }
}


/* ----------------------------- Pi Pico Onboard LED Task ------------------------------- */

void vSimpleLEDTask( void *pvParameters ){

    gpio_init( PICO_DEFAULT_LED_PIN );
    gpio_set_dir( PICO_DEFAULT_LED_PIN, GPIO_OUT );

    while( true ){

        gpio_put( PICO_DEFAULT_LED_PIN, 1 );
        vTaskDelay( 100 );
        gpio_put( PICO_DEFAULT_LED_PIN, 0 );
        vTaskDelay( 100 );
    }
}


/* --------------------------- Serial Monitor USB IO Task -------------------------------- */

/*  Task running usb serial input
    Sends task notification to vSx1280Task with pointer to 
        buffer holding the message to be sent from the sx1280 */
void vUsbIOTask( void *pvParameters ){

    uint8_t currentChar = 0x00; /* 8 bit integer to hold hex value from getchar() */

    uint8_t messageBuffer[ 256 ] = { 0 }; /* 8 bit pointer to hold outgoing message */

    /* 32 bit integer for placing input characters in the correct places in messageBuffer */
    uint32_t messageCounter = 0;

    /* Iterators */
    uint32_t i = 0;

    while( true ){

        vTaskDelay( 10 );

        currentChar = 0x00; /* Resetting the character read in from usb serial */

        /* Setting currentChar to the character being read in by getchar() */
        currentChar = getchar_timeout_us( 1000 );

        /* Checking currentChar for the error code 0xFF, < 0x20, and not a newline char */
        if( currentChar == 0xFF || ( currentChar < 0x20 && currentChar != 0x0A ) ){
            currentChar = 0x00;
        }

        /* if character read isn't "\n", and not 0x00, and less than 255 characters, index 1,
                keeping the 256th character, index 1, open for NULL, or 0x00, terminated string
           messageCounter is indexed from 0 to work with C arrays */
        if( currentChar != 0x0A && currentChar != 0x00 && messageCounter <= 254 ){ 
            /* Adding currentChar to messageBuffer at messageCounter */
            *( messageBuffer + messageCounter ) = currentChar;
            messageCounter = messageCounter + 1; /* Incrementing the value of messageCounter */
        }
        /* Checking if the character being read in is "\n" */
        else if( currentChar == 0x0A && messageCounter <= 254 ){ 

            /* Adding currentChar to the last cell in the pointer array */
            *( messageBuffer + 255 ) = 0x00; /* Adding 0x00 to NULL terminate input string */

            /* for( i = 0; i <= 255; i++ ){
                printf( "Typed Message: 0x%X %c %i\n", *( messageBuffer + i ), *( messageBuffer + i ), i );
            } */

            /* Checking to see if messageBuffer begins with "SD:"
               Messages begining with "SD:" are SD commands to be sent to vSdCardTask */
            if( *( messageBuffer ) == 0x53 && *( messageBuffer + 1 ) == 0x44 && *( messageBuffer + 2 ) == 0x3A ){

                /* FreeRTOS function updating a receiving task’s notification value */
                xTaskNotify(
                    xSdCardTaskHandle,                /* TaskHandle_t xTaskToNotify */ 
                    ( uint32_t ) messageBuffer,       /* (int)&buffer[0] */
                    eSetValueWithOverwrite );         /* eNotifyAction eAction */ 
            }
            else{ /* Otherwise the message is sent to vSx1280Task */

                /* FreeRTOS function updating a receiving task’s notification value 
                   Nolan Roth helped pay in marinara*/
                xTaskNotify(
                    xSx1280TaskHandle,                /* TaskHandle_t xTaskToNotify */ 
                    ( uint32_t ) messageBuffer,       /* (int)&buffer[0] */
                    eSetValueWithoutOverwrite );      /* eNotifyAction eAction */
            }
            vTaskDelay( 2000 ); /* To allow other tasks time to grab the notification values */
            memset( messageBuffer, 0, 256 ); /* Reassign all values in messageBuffer to 0 */
            messageCounter = 0; /* Reassign messageCounter to 0 to restart input cycle */
        }
        else if( messageCounter > 254 ){ /* if the message is more than 255 characters */

            printf( "Message is too large, must be 255 characters or less! Try Again\n" );
            messageCounter = 0;
        }
    }
}


/* ------------------------------------- MAIN ------------------------------------------- */

int main( void ){

    stdio_init_all( );

    /* TaskFunction_t pvTaskCode */
    /* const char * const pcName */
    /* uint16_t usStackDepth in words not bytes, 32 bit stack width bytes = usStackDepth * 4 */
    /* void *pvParameters */
    /* UBaseType_t uxPriority */
    /*TaskHandle_t *pxCreatedTask */

    uint32_t status = xTaskCreate(
                    vSimpleLEDTask,  
                    "Green Led",    
                    1024,           
                    NULL,           
                    1,              
                    &xSimpleLEDTaskHandle ); 

    uint32_t ioStatus = xTaskCreate(
                    vUsbIOTask,             /* TaskFunction_t pvTaskCode    */
                    "Simple IO",            /* const char * const pcName    */
                    1024,                   /* uint16_t usStackDepth        */
                    NULL,                   /* void *pvParameters           */
                    1,                      /* UBaseType_t uxPriority       */
                    &xUsbIOTaskHandle );    /* TaskHandle_t *pxCreatedTask  */

    uint32_t sx1280Status = xTaskCreate(
                    vSx1280Task,
                    "sx1280",
                    4096, /* usStackDepth * 4 = stack in bytes, because pico is 32 bits wide */
                    NULL,
                    1,
                    &xSx1280TaskHandle );

    vTaskStartScheduler();

    while(true){

    }

    return 0;
}
