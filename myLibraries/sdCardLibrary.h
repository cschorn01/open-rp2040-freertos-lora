#ifndef SDCARDLIBRARY_H
#define SDCARDLIBRARY_H

void sdInitialize( );

void sdDataRead( );

uint8_t sdCRC7( uint8_t *sdCommand );

uint32_t bootChecksum( uint8_t *sectors, uint16_t bytesPerSector );



#endif
