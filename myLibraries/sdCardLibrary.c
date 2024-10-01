
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

#include "sdCardLibrary.h"

/* ------------ Defining SD Card Command Index with Hexadecimel Commands ------------ */

/* SD Card Command Format
Desctription   Start Bit   Transmission Bit    Command Index   Argument    CRC7    End Bit
Bit Position     [47]          [46]              [45:40]       [39:8]     [7:1]     [0]
Value             '0'           '1'                 x             x         x        x
Bit Width          1             1                  6             32        7        1  */

/* ALL COMMANDS ARE PROTECTED BY CRC CHECK SECTION 4.5 AND PUT INFO AND
        PUT RESPONSES IN SECTION 7 AT THE BOTTOM

   ALL COMMANDS ARE DEFINED WITH AN 8 BIT HEXADECIMAL NUMBER, WHICH INCLUDES
        THE START, TRANSMISSION, AND COMMAND INDEX BITS
   THE ARGUMENT AND CRC WILL BE SENT IN 8 BIT DATA CHUNKS */

/* CMD0 
   Resets the SD memory card
   Since there are no arguments the CRC field is constant and
        therefore "0x40 0x00 0x00 0x00 0x00 0x95" is a valid command
   Argument: [31:0] Stuff bits
   Response: R1 */
#define GO_IDLE_STATE 0x40 

/* CMD1
   Sends host capacity support information and activates 
        the card's initialization process
   HCS is effective when card receives SEND_IF_COND command
   Reseved bits shall be set to 0
   CMD1 is a valid command for the Thim (1.4mm) Standard Size SD Memory Card
        only if used after re-initializating a card (not after power on reset)
   Argument:   [31] Reserved Bit
               [30] HCS (Host Capacity Support)
             [29:0] Reserved Bits
   Response: R1 */
#define SEND_OP_COND 0x41

/* CMD6
   Checks switchable function (mode 0) and switches card function (mode 1)
   Go to section 4.3.10 and see what info can be put here
   Argument: [31] Mode 0: Check Function 
                  Mode 1: Switch Function
             [30:24] reserved (All 0)
             [23:20] reserved for function group 6 (All 0 or 0xF)
             [19:16] reserved for function group 5 (All 0 or 0xF)
             [15:12] reserved for function group 4 (all 0 or 0xF)
              [11:8] reserved for function group 3 (all 0 or 0xF)
               [7:4] function group 2 for command system
               [3:0] function group 1 for access mode
   Response: R1 */
#define SWITCH_FUNC 0x46

/* CMD8
   Sends SD memory card interface condition, which includes host 
        supply voltage information and asks the card whether it can operate
        in the supplied voltage range
   Reserved bits shall be set to 0
   Only one bit of supply voltage shall be set to 1 at any given time
   This command is added in spec version 2.00, not compatible with ealier versions
        0b0000: Not Defined
        0b0001: 2.7-3.6V
        0b0010: Reserved for Low Voltage Range
        0b0100: Reserved
        0b1000: Reserved
        Others: Nother Defined
   Argument: [31:12] Reserved bits
              [11:8] supply voltage (VHS)
               [7:0] check pattern
   Response: R7 */
#define SEND_IF_COND 0x48

/* CMD9
   Asks the selected card to send its card-specific data (CSD)
   Argument: [31:0] stuff bits
   Response: R1 */
#define SEND_CSD 0x49

/* CMD10
   Asks the selected card to send its card identification (CID)
   Argument: [31:0] stuff bits
   Response: R1 */
#define SEND_CID 0x4A

/* CMD12
   Forces the card to stop transmission Multiple Block Read Operation
   Argument: [31:0] stuff bits
   Response: R1b (R1 rseponse with an optional trailing busy signal) */
#define STOP_TRANSMISSION 0x4C

/* CMD13
   Asks the card to send its status register
   Argument: [31:0] stuff bits 
   Response: R2 */
#define SEND_STATUS 0x4D

/* CMD16
   In case of SDSC card, block length is set by this command
   In case of SDHC and SDXC cards, block length of the memory access commands
        are fixed to 512 bytes
   The length of LOCK_UNLOCK command is set by this command regardless of
        card capacity
   Argument: [31:0] block length
   Response: R1 */
#define SET_BLOCKLEN 0x50

/* CMD17
   Reads a single block of the size set by SET_BLOCKLEN from the card
   The data transferred shall not cross a physical block boundary unless
        READ_BLK_MISALIGN is set in the CSD
   SDSC card (CCS=0) uses btyte unit address and SDHC and SDXC Cards (CCS=1)
        use block unit address (512 bytes)
   Arugment: [31:0] data address
   Response: R1 */
#define READ_SINGLE_BLOCK 0x51

/* CMD18
   Continuously transfers data blocks from card to host until 
        interrupted by a STOP_TRANSMISSION command
   SDSC card (CCS=0) uses byte unit address and SDHC and SDXC Cards (CCS=1)
        use block unit address (512 bytes)
   Argument: [31:0] data address
   Response: R1 */
#define READ_MULTIPLE_BLOCK 0x52

/* CMD24
   Writes a block of the size set by the SET_BLOCKLEN command
   The data transferred shall not cross a physical block boundary unless
        WRITE_BLK_MISALIGN is set in the CSD
   SDSC card (CCS=0) uses btyte unit address and SDHC and SDXC Cards (CCS=1)
        use block unit address (512 bytes)
   Argument: [31:0] data address
   Rseponse: R1 */
#define WRITE_BLOCK 0x58

/* CMD25
   Continuously writes blocks of data until 'Stop Tran' token is
        sent (instead of 'start block')
   SDSC card (CCS=0) uses btyte unit address and SDHC and SDXC Cards (CCS=1)
        use block unit address (512 bytes)
   Argument: [31:0] data address
   Response: R1 */
#define WRITE_MULTIPLE_BLOCK 0x59

/* CMD27
   Programing the programmable bits of the CSD
   Argument: [31:0] stuff bits
   Response: R1 */
#define PROGRAM_CSD 0x5B

/* CMD28
   If the card has write protection features, this command sets the wrie
        protection but of the address group
   The properties of write protection are coded in the card 
        specific data (WP_GRP_SIZE)
   SDHC and SDXC do not support this command
   Argument: [31:0] data address
   Response: R1b  (R1 rseponse with an optional trailing busy signal) */
#define SET_WRITE_PROT 0x5C

/* CMD29
   If the card har write protection features, this command clears the 
        write protection bit of the addressed group
   SDHC and SDXC do not support this command
   Argument: [31:0] data address
   Response: R1b (R1 rseponse with an optional trailing busy signal) */
#define CLR_WRITE_PROT 0x5D

/* CMD30
   If the card has write proteciton features this command, asks the
        card to send the status of the write protection bits
   32 write protection bits (represtnting 32 write protect groups
        at the specified address) followed by 16 CRC bits corresponds
        to the first addressed group. If the addresses of the last
        groups are outside the valid range, then the corresponding
        wrie protection bits shall be set to zero
   SDHC and SDXC do not support this command
   Argument: [31:0] write protect data address
   Rseponse: R1 */
#define SEND_WRITE_PROT 0x5E 

/* CMD32
   Sets the address of the first write block to be erased
   SDSC card (CCS=0) uses btyte unit address and SDHC and SDXC Cards (CCS=1)
        use block unit address (512 bytes)
   Argument: [31:0] data address
   Response: R1 */
#define ERASE_WR_BLK_START_ADDR 0x60 

/* CMD33
   Sets the address of the last write block of the continuous 
        range to be erased 
   SDSC card (CCS=0) uses btyte unit address and SDHC and SDXC Cards (CCS=1)
        use block unit address (512 bytes)
   Argument: [31:0] data address
   Response: R1 */
#define ERASE_WR_BLK_END_ADDR 0x61 

/* CMD38
   Erases all previously selected write blocks
   FULE and DISCARD are not supported through SPI interface
   Argument: [31:0] stuff bits
   Response: R1b (R1 rseponse with an optional trailing busy signal) */
#define ERASE 0x66 

/* CMD55
   Indicates to the card that the next command is an application
        specific command rather than a standard command
   Argument: [31:0] stuff bits
   Response: R1 */ 
#define APP_CMD 0x77

/* CMD56
   Used either to transfer a Data Block to the card or to get a
        Data Block from the card for general purpose/application
        specific commands
   In case of Standard Capacity SD memory Card, the size of the 
        Data Block shall be defined with SET_BLOCK_LEN command
   In case of SDHC and SDXC Cards, block leng of this command
        is fixed to 512-byte
   Argument: [31:0] stuff bits
                [0] RD/WR 0: the host sends a block of data to the card 
                          1: the host shall get a block of data from the card
   Response: R1 */
#define GEN_CMD 0x78

/* CMD58
   Reads the OCR register of a card
   CCS (Card Capacity Status) bit is assigned to OCR[30]
        CCS = 0 means its an SDSC card and CCS = 1 means its and SDHC or SDXC card
   Argument: [31:0] stuff bits
   Response: R1 */
#define READ_OCR 0x7A

/* CMD59
   Turns the CRC option on or off
   A '1' in the CRC option bit will the option on, and a '0'
        will turn it off
   Argument: [31:1] stuff bits
                [0] CRC option
   Response: R1 */
#define CRC_ON_OFF 0x7B

/* ACMD13
   Sends the SD Status
   Status fields are given in table 4-44 in simplified physical layer spec
   Argument: [31:0] stuff bits
   Response: R2 */
#define SD_STATUS 0x0D

/* ACMD22
   Sends the number of the well written (without errors) blocks
   Responds with 32bit+CRC data block
   Argument: [31:0] stuff bits
   Response: R1 */
#define SEND_NUM_WR_BLOCKS 0x0E

/* ACMD23
   Sets the number of write blocks to be pre-erased before writing
   To be used for faster WRITE_MULTIPLE_BLOCK commands
   1 = default (one write block)
   Stop Tran Toekn shall be used to stop the tansmission in 
        WRITE_MULTIPLE_BLOCK whether the pre-rease (ACMD23) feature
        is used or not
   Argument: [31:23] stuff bits
              [22:0] Number of blocks
   Response: R1 */
#define SET_WR_BLK_ERASE_COUNT 0x0F

/* ACMD41
   Sends host capacity support information (HCS) and acitvates the
        cards initialiation process
   Reserved bits shall be set to 0
   Argument:   [31] Reserved bit
               [30] HCS (Host Capacity Support)
             [29:0] Reserved bits
   Response: R1 */
#define SD_SEND_OP_COND 0x29

/* ACMD42
   Connects[1]/Disconnects[0] the 50 KOhm pull-up resistor on 
        CS (pin 1) of the card
   The pull up may be used for card detection
   Argument: [31:1] stuff bits
                [0] set_cd */
#define SET_CLR_CARD_DETECT 0x2A

/* ACMD51
   Reads the SD Configuration Register (SCR)
   Argument: [31:0] stuff bits
   Response: R1 */
#define SEND_SCR 0x33

/* SD CARD RESPONSES

   R1 Format
   1 byte [7:0]
   Bit#:  Flag Meaning
    [7]:  Reserved, always 0
    [6]:  Parameter Error: The Command's argument (e.g. address, block length was
                           outside the allowed range for this card
    [5]:  Address Error: A misaligned address that did not match the block lenght
                         was used in the command
    [4]:  Erase Sequence Error: An error in the sequence of erase commands occurred
    [3]:  Communication CRC error: The CRC check of the last command failed
    [2]:  Illegal Command: An illegal command code was detected
    [1]:  Erase Reset: An erase sequence was cleared before executing because of an
                       out of erase sequence command was received
    [0]:  In idle state: The card is in idle state and running the initialization
                         process

   R1b Format
   Identical to the R1 format with the optional addition of the bust signal
   Busy signal token can be any number of bytes
   Zero indicates card is busy
   Non-zero value indicates the card is readt for the next command

   R2 Format
   2 bytes
   First byte is identical to R1
   Sent as a response to the SEND_STATUS command
   Bit#:   Flag Meaning
    [15]:  Reserved, always 0
    [14]:  Parameter Error: The Command's argument (e.g. address, block length was
                           outside the allowed range for this card
    [13]:  Address Error: A misaligned address that did not match the block length
                         was used in the command
    [12]:  Erase Sequence Error: An error in the sequence of erase commands occurred
    [13]:  Communication CRC error: The CRC check of the last command failed
    [10]:  Illegal Command: An illegal command code was detected
    [9]:  Erase Reset: An erase sequence was cleared before executing because of an
                       out of erase sequence command was received
    [8]:  In idle state: The card is in idle state and running the initialization 
                         process
    [7]:  Out of range | csd overwrite: No description on documentation
    [6]:  Erase Param: An invalid selection for erase, sectors or groups
    [5]:  Write Protect Violation: The command tried to write a write-protected block
    [4]:  Card ECC Failed: Card internal ECC wass applied but failed to 
                           correct the data
    [3]:  CC Error: Internal card controller error
    [2]:  Error: A general or an unkown error occurred during the operation
    [1]:  Write protect erase skip | lock/unlock command failed:
                 This status bit has two functions overloaded. It is set when the host
                 attempts to erase a write-protected sector or makes a sequence or
                 password errors during card lock/unlock operation
    [0]:  Card is Locked: Set when the card is locked by the user
                          Reset when it is unlocked

   R3 Format
   5 bytes
   First byte is identical to R1
   Other 4 bytes contain the OCR Register
   Sent by the card when a READ_OCR command is received

   R4 & R5 Formats
   Reserved for I/O mode ( refer to "SDIO Card Specification" )

   R7 Format
   5 bytes
   First byte is identical to R1
   Other 4 bytes contatin the card operating voltage and echo back of check pattern
   Sent by the card when a SEND_IF_COND (CMD8) command */

/* exFAT File System

   VOLUME STRUCTURE
   Field Name                       Offset      Size
                                   (sector)     (sectors)
   MAIN BOOT REGION
   Main Boot Sector                   0          1
   Main Extended Boot Sectors         1          8
   Main OEM Parameters                9          1
   Main Reserved                      10         1
   Main Boot Checksum                 11         1

   BACKUP BOOT REGION
   Backup Boot Sector                 12         1
   Backup Extended Boot Sectors       13         8
   Backup OEM Parameters              21         1
   Backup Reserced                    22         1
   Backup Boot Checksum               23         1

   FAT REGION
   Fat Alignment                      24         FatOffset-24
   First FAT                          FatOffset  FatLength
   Second FAT                         ( FatOffset + FatLength )
                                                 ( FatLength * ( NumberOfFats - 1 ) )

   DATA REGION
   Cluster Heap Alignment             ( FatOffset + FatLength * NumberOfFats )
                                                 ( ClusterHeapOffset – (FatOffset + FatLength * NumberOfFats) )
   Cluster Heap                       ClusterHeapOffset
                                                 ( ClusterCount * ( 2^SectorsPerClusterShift ) )
   Excess Space                       ( ClusterHeapOffset + ClusterCount * ( 2^SectorsPerClusterShift ) )
                                                 ( VolumeLength – (ClusterHeapOffset + ClusterCount * ( 2^SectorsPerClusterShift ) ) )

   MAIN BOOT SECTOR
   Field Name       Offset  Size    Comments
                    (byte)  (bytes)

   JumpBoot           0       3     Contain the jump instruction for CPUs common
                                    in personal computers, which, when executed,
                                    "jumps" the CPU to execute the boot-strapping
                                    instructions in the BootCode field

   FileSystemName    3        8    Contain the name of the file system on 
                                   the volume. Valid value is ASCII characters 
                                   "EXFAT ", including three trailing 
                                   white spaces
        
   MustBeZero        11       53   Correspond with the range of bytes the packed
                                   BIOS parameter block consumes on FAT12/16/32
                                   volumes. Valid value for this field is 0, 
                                   which helps to prevent FAT12/16/32
                                   implementations from mistakenly mounting 
                                   an exFAT volume

   PartitionOffset   64       8    Describes media-relative sector offset of
                                   the partition hosting the exFAT volume. 
                                   All possible values for this field are valid;
                                   the value 0 indicates implementations
                                   shall ignore this field

   VolumeLength      72       8    Describes the size of the given exFAT volume
                                   in sectors. Valid range of values for this field
                                   shall be at least (2^20)/(2^BytesPerSectorShift),
                                   at most (2^64)-1

   FatOffset         80       4    Describe the volume-relative sector offset
                                   of the First FAT. Enables implementations to align
                                   the First FAT to the characteristics of the
                                   underlying storage media. Valid range of values
                                   for this field shall be at least 24 and at
                                   most (ClusterHeapOffset - (FatLength * 
                                   NumberOfFats)) 

   FatLength         84       4    Describe the length, in sectors, of each FAT table.
                                   Valid range of values for this field shall be at
                                   least ((ClusterCount + 2) * (2^2) /
                                   (2%BytesPerSectorShift) ) rounded up to the
                                   nearest integer ensuring each FAT has sufficient
                                   space for describing all cluster in the
                                   Cluster Heap and at most 
                                   ( (ClusterHeapOffset - FatOffset) / NumberOfFats )
                                   rounded to the nearest integer ensuring the FAT's
                                   exit before the Cluster Heap

   ClusterHeapOffset 88       4    Describes the volume-relative sector offset of 
                                   the Cluster Heap. Enables implementations to align
                                   the Cluster Heap to the characteristics of the 
                                   underlying storage media. Valid range of values are
                                   at lest ( FatOffset + FatLength * NumberOfFats ),
                                   to account for the sectors all the preceding
                                   regions consume, and at most ( (2^32)- 1 ) or
                                   ( VolumeLength - (ClusterCount *
                                   2SectorsPerClusterShift) ), whichever 
                                   calculation is less

   ClusterCount      92       4    Describes the number of clusters the Cluster Heap
                                   contains. The valid value for this field will be
                                   the lesser of ( (VolumeLength - ClusterHeapOffset)
                                   / (2^SectorsPerClusterShift) ) rounded down to
                                   nearest integer, which is exactly the number of
                                   clusters that can fit between the beginning of 
                                   the Cluster Heap and the end of the volume, and 
                                   (2^32) - 11, which is the maximum number of 
                                   clusters a FAT can describe

   FirstClusterOfRootDirectory     Contain the cluster index of the first cluster of
                                   the root directory. Implementations should make
                     96       4    every effort to place the first cluster of the 
                                   root directory in the first non-bad cluster after 
                                   the clusters the Allocation Bitmap and Up-case 
                                   Table consume. The valid range of values for this
                                   field shall be at least 2, the index of the first
                                   cluster in the Cluster Heap, and at most
                                   ( ClusterCount + 1 ), the index of the last 
                                   cluster in the Cluster Heap

   VolumeSerialNumber              Contain a unique serial number. This assists 
                     100      4    implementations to distinguish among different 
                                   exFAT volumes. Implementations should generate 
                                   the serial number by combining the date and time 
                                   of formatting the exFAT volume. The mechanism for 
                                   combining date and time to form a serial number 
                                   is implementation-specific. All possible values 
                                   for this field are valid.

   FileSystemRevision              Describes the major and minor revision numbers of 
                                   the exFAT structures on the given volume. The 
                     104      2    high-order byte is the major revision number and 
                                   the low-order byte is the minor revision number,
                                   i.e. 0x01 and 0x05 describes revision number 1.05.
                                   The valid range of values for this field are at 
                                   least 0 for the low-order byte and 1 for the 
                                   high-order byte and at most 99 for the low-order
                                   byte and 99 for the high-order byte. The revision
                                   number of exFAT this specification describes
                                   is 1.00.

   VolumeFlags        106     2    Contain flags which indicate the status of various
                                   file system structures on the exFAT volume. 
                                   Implementations shall not include this field when 
                                   computing its respective Main Boot or Backup Boot 
                                   region checksum. 
                                   -------------------------------------------------
                                   Field Name 	Offset   Size   Comments
                                                (bit)   (bits)
	                               ActiveFat      0        1    HAVE TO FIGURE 
                                                                THESE COMMENTS OUT
                                   VolumeDirty    1        1
                                   MediaFailure   2        1
                                   ClearToZero    3        1
                                   Reserved       4       12

   BytesPerSectorShift             Describes bytes per sector expressed as log_2(N),
                     108      1    N is the number of bytes per sector. At least 
                                   9 (sector size of 512 bytes),At most 12 (sector 
                                   size of 4096 bytes).
                        

   SectorsPerClusterShift          Describes the sectors per cluster expressed as 
                     109      1    log_2(N), where N is number of sectors per cluster.
                                   At least 0 (1 sector per cluster), which is the
                                   smallest cluster possible, and at most 
                                   25 - BytesPerSectorShift, which evaluates to a 
                                   cluster size of 32MB.
                     

   NumberOfFats      110      1    Describe the number of FATs and Allocation Bitmaps
                                   the volume contains. Valid range of values for this
                                   field shall be 1, which indicates the volume only 
                                   contains the First FAT and First Allocation Bitmap,
                                   or 2, which indicates the volume contains the First
                                   FAT, Second FAT, First Allocation Bitmap, and 
                                   Second Allocation Bitmap; this value is only valid
                                   for TexFAT volumes

   DriveSelect       111      1    Contains the extended INT 0x13 drive number, which
                                   aids boot-strapping from this volume using extended
                                   INT 0x13 on personal computers. All possible values
                                   for this field are valid. Similar fields in 
                                   previous FAT-based file systems frequently 
                                   contained the value 0x80. 

   PercentInUse      112      1    Describes the percentage of clusters in the Cluster
                                   Heap which are allocated. Valid range of values for
                                   this field shall be between 0 and 100 inclusively,
                                   which is the percentage of allocated clusters in 
                                   the Cluster Heap, rounded down to the nearest 
                                   integer, or exactly 0xFF, which indicates the 
                                   percentage of allocated clusters in the Cluster 
                                   Heap is not available. Change the value of this
                                   field to reflect changes in the allocation of 
                                   clusters in the Cluster Heap or shall change 
                                   it to 0xFF. Not included in checksum.

   Reserved          113      7    This field is mandatory and its contents 
                                   are reserved. 

   BootCode          120      390  Contain boot-strapping instructions. 
                                   Implementations may populate this field with the
                                   CPU instructions necessary for boot-strapping a 
                                   computer system. Implementations which don't 
                                   provide boot-strapping instructions shall 
                                   initialize each byte in this field to 0xF4, (the
                                   halt instruction for CPUs common in personal     
                                   computers)

   BootSignature     510      2    Describes whether the intent of a given sector is
                                   for it to be a Boot Sector or not. Valid value for
                                   this field is 0xAA55. Any other value in this field
                                   invalidates its respective Boot Sector. 
                                   Implementations should verify the contents of this
                                   field prior to depending on any other field in its
                                   respective Boot Sector.

   ExcessSpace       512      (2^BytesPerSectorShift) - 512 ) This field is mandatory                                                                         and its contents, if any,
                                                              are undefined.


   FILE ALLOCATION TABLE REGION
   Field Name       Offset  Size    Comments
                    (byte)  (bytes)
   FatEntry[0]         0      4     Describes the media type in the first byte
                                    (the lowest order byte) and shall contain 0xFF
                                    in the remaining three bytes. The first byte 
                                    should be 0xF8

   FatEntry[1]         4      4     Only exists due to historical precedence and does
                                    not describe anything of interest. Valid value for
                                    this field is 0xFFFFFFFF

   FatEntry[2]         8      4     Each FatEntry field in this array shall represent
                                    a cluster in the Cluster Heap. FatEntry[2] 
                                    represents the first cluster in the Cluster Heap 
                                    and FatEntry[ClusterCount+1] represents the last 
                                    cluster in the Cluster Heap. Valid range of values                                    for these fields shall be:
                                    Between 2 and ClusterCount + 1, inclusively, which
                                    points to the next FatEntry in the given cluster 
    ...                             chain; the given FatEntry shall not point to any
                                    FatEntry which precedes it in the given cluster 
                                    chain.
                                    Exactly 0xFFFFFFF7, which marks the given 
                                    FatEntry's corresponding cluster as "bad".
                                    Exactly 0xFFFFFFFF, which marks the given 
                                    FatEntry's corresponding cluster as the last 
                                    cluster of a cluster chain; this is the only valid
                                    value for the last FatEntry of any given cluster
                                    chain.

   

   FatEntry[ClusterClount+1]  4
                       (ClusterCount+1)*4

   Excess Space        (ClusterCount+1)*4
                              (FatLength * (2^BytesPerSectorShift)) – ((ClusterCount + 2) * 4)

   
   DATA REGION
   Contains the Cluster Heap, which provides managed space for file system structures,
        directories, and files.
   Each consecutive series of sectors describes one cluster, as the 
        SectorsPerClusterShift field defines.
   Importantly, the first cluster of the Cluster Heap has index two, which directly
        corresponds to the index of FatEntry[2].
   Field Name       Offset  Size    Comments
                    (byte)  (bytes)                         Each Cluster field in this
   Cluster[2]         ClusterHeapOffset                     array is a series of 
                              (2^SectorsPerClusterShift)    contiguous sectors, whose
                                                            size is defined by the 
   ...                                                      SectorsPerClusterShift 
                                                            field.

   Cluster[ClusterCount+1]    (2^SectorsPerClusterShift)
                      ClusterHeapOffset + (ClusterCount – 1) * 2SectorsPerClusterShift
   

*/


/*  Driving the chip select pin low 
    Transactions with SD Card start with chip select low */
void sdCardSelect(){

    asm volatile ("nop \n nop \n nop");/* Find out what it does */
    gpio_put( 17, 0 );
    asm volatile ("nop \n nop \n nop");

}


/*  Driving the chip select pin high 
    Transactions with SD Card end with chip select high */
void sdCardDeselect(){

     asm volatile ("nop \n nop \n nop");
     gpio_put( 17, 1 );
     asm volatile ("nop \n nop \n nop");

}


/* Function performing a checksum on Main and Backup Boot sub-regions in the exFat file system
   This funcion is written by Microsoft and is found in Exfat-specification Section 3.4
   Checksum calculation does not include the VolumeFlags and PercentInUse fields */
uint32_t bootChecksum( uint8_t *sectors, uint16_t bytesPerSector ){

    uint32_t numberOfBytes = ( uint32_t ) bytesPerSector * 11;
    uint32_t checksum = 0; 
    uint32_t i = 0;

    for( i = 0; i < numberOfBytes; i++ ){
        if( i == 106 || i == 107 || i == 112 ){
            continue;
        }
        checksum = ( ( checksum & 1 ) ? 0x80000000 : 0 ) + ( checksum >> 1 ) + ( uint32_t ) *( sectors + i );
    }

    return checksum;
}


void sdCRC16( uint8_t *sdData, uint32_t n ){

    uint16_t crc = 0;

    for (size_t i = 0; i < n; i++) {
        crc = (uint8_t)(crc >> 8) | (crc << 8);
        crc ^= sdData[i];
        crc ^= (uint8_t)(crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xff) << 5;
    }

    /* return crc; */
}


/* Function creating an 8 bit hexadecimal number meant for filling the last 8 bits
        of the SD command format which includes a 7 bit CRC and an end bit (always 1)
   Returns a single byte to the 6th byte of a command pointer array to fill the CRC & end bit */
uint8_t sdCRC7( uint8_t *sdCommand ){

    /* Generator Polynomial: G(x) = x^7 + x^3 + 1 or 0b10001001 */

    uint8_t crc = 0; /* *( sdCommand ); */

    uint8_t commandByteHolder = 0x00;

    /* Iterator */
    uint8_t i = 0;

      /* Most recent attempt at CRC Calculation, close but unsuccessful */
//    for( uint8_t i = 8; i < 40; i++){ /* Looping through SD Command bits for CRC calculation */
//        crc = ( crc << 1 ) | ( *( sdCommand + i/8 ) >> ( ( i - ( i/8 * 8 ) ) & 0x01 ) );
//        printf( "CRC: 0x%X %i\n", crc, i );
//        if( ( crc & 0x80 ) == 0x80 ){ /* Checking the first bit in the CRC for a 1 */
            /* Xor with generator polynomial, G(x) = x^7 + x^3 + 1 or 0b10001001*/
//            crc = crc ^ 0x89; 
//        }
        /* Operation shifting SD Command bits individually from 8 bit pointer values
           Left shift crc 1 bit, right shift SD Command till the ith bit is in the right
                most bit position of the 8 bit pointer value, these two are then "or"ed */
        /* crc = ( crc << 1 ) | ( *( sdCommand + i/8 ) >> ( ( 8*( i/8 + 1 ) - i ) & 0x01 ) ); */
//    }

    /* From SDFat Arduino library at github.com/greiman/SdFat, I dont quite understand CRC yet*/
    for( i = 0; i < 5; i++ ){
        commandByteHolder = *( sdCommand + i ); 
        for( uint8_t j = 0; j < 8; j++ ){
            crc = crc << 1;
            if( ( commandByteHolder & 0x80 ) ^ ( crc & 0x80 ) ){
                crc = crc ^ 0x09;
            }
            commandByteHolder = commandByteHolder << 1;
        }
    }

    crc = ( crc << 1 ) | 0x01;

    /* printf( "CRC: 0x%X\n", crc ); */

    return crc;
}




void sdInitialize( ){

    /* 8 bit poiner address for dynamic arrays */
    uint8_t sdCommand[ 20 ] = { 0 };
    uint8_t sdResponse[ 20 ] = { 0 };

    /* 8 bit integer holding a boolean value indicating useability of connected SD Card */
    uint8_t isSdCardUseable = 0;

    /* 8 bit integer holding a boolean value, mirrors CMD8 response bit [34] */
    uint8_t isSdCardVersion2 = 0;

    /* 8 bit boolean integer, mirrors OCR register bit [30], SDSC(0) or an SDHC/SDXC/SDUC(1) */
    uint8_t cardCapacitySupport = 0;

    /* Iterators */
    uint32_t i = 0;
    uint32_t j = 0;
    uint32_t k = 0;

    /* Pre initialize, SPI0 TX GP19 is 1, CS CP17 is 1, SCK GP18 >74 clock pulses */
    for( i = 0; i < 10; i++ ){
        *( sdCommand + i ) = 0xFF;
    }
    sdCardDeselect();
    spi_write_read_blocking( spi0, sdCommand, sdResponse, 8*sizeof( uint8_t ) );
    for( i = 0; i < 10; i++){
        printf( "SD Response after >74 Clock Cycles: 0x%X\n", *( sdResponse + i ) );
    }

    /* The first command over SPI is CMD0
       GO_IDLE_STATE ( CMD0 ) responds with an R1 response which is 1 byte */
    *( sdCommand ) = GO_IDLE_STATE;
    *( sdCommand + 1 ) = 0x00; 
    *( sdCommand + 2 ) = 0x00; 
    *( sdCommand + 3 ) = 0x00; 
    *( sdCommand + 4 ) = 0x00; 
    *( sdCommand + 5 ) = sdCRC7( sdCommand ); /* 0x95; */
    for( i = 6; i < 20; i++){
        *( sdCommand + i ) = 0xFF;
    }
    sdCardSelect();
    spi_write_read_blocking( spi0, sdCommand, sdResponse, 20*sizeof( uint8_t ) );
    sdCardDeselect();
    for( i = 0; i < 20; i++){
        printf( "CMD0 sdResponse: 0x%X %i\n", *( sdResponse + i ), i );
        if( *( sdResponse + i ) == 0x01 ){ /* If it's 0x01 sd card is in idle state */
            break;
        }
    }

    /* CMD8, 0b0001 in Supply Voltage bits [11:8], or 2.7-3.6 Volts in SD PHY Layer Doc
       SEND_IF_COND( CMD8 ) responds with an R7 response which is 40 bits */
    *( sdCommand ) = SEND_IF_COND;
    *( sdCommand + 1 ) = 0x00; /* Reserved [31:24] */ 
    *( sdCommand + 2 ) = 0x00; /* Reserved [23:16] */ 
    *( sdCommand + 3 ) = 0x01; /* Reserved [15:12] Supply Voltage [11:8] */ 
    *( sdCommand + 4 ) = 0x00; /* Check Pattern [7:0] */ 
    *( sdCommand + 5 ) = sdCRC7( sdCommand );
    for( i = 6; i < 20; i++){
        *( sdCommand + i ) = 0xFF;
    }
    sdCardSelect();
    spi_write_read_blocking( spi0, sdCommand, sdResponse, 20*sizeof( uint8_t ) );
    sdCardDeselect();
    for( i = 0; i < 20; i++){ 
        printf( "CMD8 sdResponse: 0x%X %i\n", *( sdResponse + i ), i );
        if( *( sdResponse + i ) == 0x01 ){ /* If it's 0x01 sd card is in idle state */
            break;
        } 
    }

    /* Checking illegal command bit [34] flag isn't 1, and the Voltage range is correct
       Using i from last for loop to keep the place of where the CMD8 Response begins
       For some reason the echo back crc is always 0x00 instead of the CMD8 CRC */
    if( *( sdResponse + i ) & 0x01 == 0x01 && ( *( sdResponse + i + 3 ) & 0x01 ) == 0x01 ){

        /* Illegal Command Bit [34] is 0 from CMD8, SD Card is version 2.00 or later */
        isSdCardVersion2 = 1;

        /* CMD59 toggles CRC option on or off, '1' turns CRC on, '0' off
           CRC_ON_OFF( CMD59 ) responds with an R1 response, which is 1 byte */
        *( sdCommand ) = CRC_ON_OFF;
        *( sdCommand + 1 ) = 0x00; /* Stuff Bits [31:24] */ 
        *( sdCommand + 2 ) = 0x00; /* Stuff Bits [23:16] */ 
        *( sdCommand + 3 ) = 0x00; /* Stuff Bits [15:8] */ 
        *( sdCommand + 4 ) = 0x01; /* Stuff Bits [7:1], CRC Option [0] */ 
        *( sdCommand + 5 ) = sdCRC7( sdCommand );
        for( i = 6; i < 20; i++){
            *( sdCommand + i ) = 0xFF;
        }
        sdCardSelect();
        spi_write_read_blocking( spi0, sdCommand, sdResponse, 20*sizeof( uint8_t ) );
        sdCardDeselect();
        for( i = 0; i < 20; i++){
            printf( "CMD59 sdResponse: 0x%X %i\n", *( sdResponse + i ), i );
            if( *( sdResponse + i ) == 0x01 ){ /* If it's 0x01 sd card is in idle state */
                break;
            }
        }

        /* CMD58 requests SD Card OCR register, bit [20] is 3.2-3.3V, [21] is 3.3-3.4V 
           READ_OCR( CMD58 ) responds with an R3 response which is 40 bit */
        *( sdCommand ) = READ_OCR;
        *( sdCommand + 1 ) = 0x00; /* Reserved [31:24] */ 
        *( sdCommand + 2 ) = 0x00; /* Reserved [23:16] */ 
        *( sdCommand + 3 ) = 0x00; /* Reserved [15:8] */ 
        *( sdCommand + 4 ) = 0x00; /* Reserved [7:0] */ 
        *( sdCommand + 5 ) = sdCRC7( sdCommand );
        for( i = 6; i < 20; i++){
            *( sdCommand + i ) = 0xFF;
        }
        sdCardSelect();
        spi_write_read_blocking( spi0, sdCommand, sdResponse, 20*sizeof( uint8_t ) );
        sdCardDeselect();
        for( j = 0; j < 20; j++){
            printf( "1st CMD58 sdResponse: 0x%X %i\n", *( sdResponse + j ), j );
            if( *( sdResponse + j ) == 0x01 ){
                break;
            }
        }

        /* If either bit [20] or [21] equal 1, the SD card can use rp2040 3.3V out */
        if( *( sdResponse + j + 2 ) & 0x10 == 0x10 || *( sdResponse + j + 2 ) & 0x08 == 0x08 ){

            isSdCardUseable = 1;
            printf("CMD58 Card Useable\n");
        }
    }
    else{

        /* Checking SEND_IF_COND( CMD8 ) response bits [11:8] for the issue  */
        if( ( *( sdResponse + i + 3 ) & 0x01 ) != 0x01 ){

            printf( "Incorrect Operating Voltage you can't use this card\n" );
            isSdCardUseable = 0; 
        }

        if( *( sdResponse + i + 4 ) != *( sdCommand + 5 ) ){

            printf( "Incorrect 'echo back' CRC\n" );
        }

        /* Checking SEND_IF_COND( CMD8 ) response for the Illegal Command Bit
           The first byte of R7 is the same as R1, so bit [34] in R7 is the
                 Illegal Command Bit */
        if( *( sdResponse + i ) & 0x04 == 0x04 ){

            printf("Illegal Command Flag thrown ( bit [34] of R7 )\n");
            isSdCardVersion2 = 0;
        }
        else if( *( sdResponse + i ) & 0x04 == 0x00 ){

            printf("Illegal Command Flag NOT thrown ( bit [34] of R7 )\n");
            isSdCardVersion2 = 1;
        }
    }


    if( isSdCardUseable == 1 ){

        for( i = 0, j = 0; *( sdResponse + j ) & 0x01 == 0x01; i++ ){

            vTaskDelay( 2000 );

            /* Repeating ACMD41 till SD card isn't in an idle state */
            *( sdCommand ) = SEND_OP_COND;
            *( sdCommand + 1 ) = 0x40; /* High Capacity Support, bit [30], is 1 */
            *( sdCommand + 2 ) = 0x00; 
            *( sdCommand + 3 ) = 0x00; 
            *( sdCommand + 4 ) = 0x00; 
            *( sdCommand + 5 ) = sdCRC7( sdCommand );
            for( j = 6; j < 20; j++){
                *( sdCommand + j ) = 0xFF;
            }
            sdCardSelect();
            spi_write_read_blocking( spi0, sdCommand, sdResponse, 20*sizeof( uint8_t ));
            sdCardDeselect();
            for( j = 0; j < 20; j++){
                printf( "ACMD41 sdResponse: 0x%X %i\n", *( sdResponse + j ), j );
                if( *( sdResponse + j ) == 0x01 || *( sdResponse + j ) == 0x00 ){
                    break;
                }
            }

            if( *( sdResponse + j ) == 0x00 ){ /* If 0x00 then SD Card out of Idle State */
                break;
            }

            printf( "SD card in Idle State after ACMD41 %i\n", i );

            if( i > 10000 ){

                /* Send CMD0 to put SD Card in idle state to retry initializing
                   GO_IDLE_STATE( CMD0 ) responds with an R1 response which is 1 byte */
                *( sdCommand ) = GO_IDLE_STATE;
                *( sdCommand + 1 ) = 0x00; 
                *( sdCommand + 2 ) = 0x00; 
                *( sdCommand + 3 ) = 0x00; 
                *( sdCommand + 4 ) = 0x00; 
                *( sdCommand + 5 ) = sdCRC7( sdCommand );
                for( j = 6; j < 20; j++){
                    *( sdCommand + j ) = 0xFF;
                }
                sdCardSelect();
                spi_write_read_blocking( spi0, sdCommand, sdResponse, 20*sizeof( uint8_t ) );
                sdCardDeselect();
                for( j = 0; j < 20; j++){
                    printf( "2nd CMD0 sdResponse: 0x%X %i\n", *( sdResponse + j ), j );
                    if( *( sdResponse + j ) == 0x01 ){
                        break;
                    }
                }
                break;
            } 
        }

        /* printf(" \n isSdCardVersion2: %i \n ", isSdCardVersion2 );
           printf( "isSdCardUseable: %i \n", isSdCardUseable );
           printf( "sdResponse + j: 0x%2X \n \n ", *( sdResponse + j ) ); */

        /* Checking bit [0] in ACMD41 R1 response indicates if card is in idle mode
           CMD58 is only supported in Version 2.00 and above to give the
                Card Capacity Support indicating if the SD card is an SDSC
                or an SDHC/SDXC/SDUC */
        if( *( sdResponse + j ) == 0x00 && isSdCardVersion2 == 1 && isSdCardUseable == 1 ){

            /* CMD58 Again, reading the OCR register of an SD card
               Card Capacity Support( CCS ) is assigned to OCR bit [30] 
               The OCR also contains the Card Power Up Status Bit on bit [31]
               Card Power Up Status Bit [31] is set to LOW( 0 ) if the card has not
                    finished the power up routine
               Card Capacity Status(CCS) bit is valid only when the Card Power Up
                    Status Bit is set
                    CCS = 0 means the card is an SDSC card
                    CCS = 1 means the cards is an SDHC/SDXC/SDUC card 
               READ_OCR( CMD58 ) responds with an R3 response which is 5 bytes */
            *( sdCommand ) = SEND_OP_COND;
            *( sdCommand + 1 ) = 0x00;
            *( sdCommand + 2 ) = 0x00;
            *( sdCommand + 3 ) = 0x00;
            *( sdCommand + 4 ) = 0x00;
            *( sdCommand + 5 ) = sdCRC7( sdCommand );
            for( i = 6; i < 20; i++){
                *( sdCommand + i ) = 0xFF;
            }
            sdCardSelect();
            spi_write_read_blocking( spi0, sdCommand, sdResponse, 20*sizeof( uint8_t ));
            sdCardDeselect();
            for( i = 0; i < 20; i++){
                printf("2nd CMD58 sdResponse: 0x%X %i\n", *( sdResponse + i ), i );
                /* Checking the 2nd byte in R3 for bits [31] and [30], 0xC0 = 0b1100000
                   Bit [30] is only valid if bit [31] is HIGH */
                if( *( sdResponse + i ) == 0x00 && *( sdResponse + i + 1 ) & 0xC0 == 0xC0 ){

                    cardCapacitySupport = 1;
                    printf("Card Capacity Support: %i\n", cardCapacitySupport );
                    break;
                }
            }
            printf( "SD POWER UP AND CARD IDENTIFICATION SUCCESSFUL\n" );
        }
    }
    printf( "END OF POWER UP AND CARD IDENTIFICATION MODE\n" );
}


void sdDataRead( ){

    /* 8 bit poiner address for dynamic arrays */
    uint8_t sdCommand[ 700 ] = { 0 };
    uint8_t sdResponse[ 700 ] = { 0 };

    uint32_t bootSector = 0; /* Sector/Block the boot sector is in on inserted card */

    /* Iterators */
    uint32_t i = 0;
    uint32_t j = 0;
    uint32_t k = 0;

    /* Send CMD16 to set the byte length of each block memory access
       SET_BLOCKLEN( CMD16 ) responds with an R1 response which is 1 byte */
    *( sdCommand ) = SET_BLOCKLEN;
    *( sdCommand + 1 ) = 0x00;
    *( sdCommand + 2 ) = 0x00; 
    *( sdCommand + 3 ) = 0x02; /* 32 bit binary number = 512, 0x00000200, 2^9 */
    *( sdCommand + 4 ) = 0x00; 
    *( sdCommand + 5 ) = sdCRC7( sdCommand );
    for( i = 6; i < 20; i++){
        *( sdCommand + i ) = 0xFF;
    }
    sdCardSelect();
    spi_write_read_blocking( spi0, sdCommand, sdResponse, 20*sizeof( uint8_t ));
    sdCardDeselect();
    for( j = 0; j < 20; j++){
        printf( "CMD16 sdResponse: 0x%X %i\n", *( sdResponse + j ), j );
        if( *( sdResponse + j ) == 0x00 ){
            break;
        }
    }

    /* Link to Microsoft exFAT File System Specification: 
            https://learn.microsoft.com/en-gb/windows/win32/fileio/exfat-specification
       Prior to using the contents of either the Main or Backup Boot Sector,
            implementations shall verify their contents by validating their 
            respective Boot Checksum and ensuring all their fields are within 
            their valid value range */

    for( i = 0; bootSector == 0; i++ ){ /* Looping sectors for boot sector    */

        vTaskDelay( 10 );


        /* Sending CMD17 to get the Main Boot Sector
           SDSC uses byte address, SDHC & SDXC uses block addresses
           May change to CMD18 later to grab Main Boot, Backup Boot, and FAT regions */
        *( sdCommand ) = READ_SINGLE_BLOCK;
        *( sdCommand + 1 ) = ( uint8_t ) ( ( i & 0xFF000000 ) >> 24 );
        *( sdCommand + 2 ) = ( uint8_t ) ( ( i & 0x00FF0000 ) >> 16 );
        *( sdCommand + 3 ) = ( uint8_t ) ( ( i & 0x0000FF00 ) >> 8 );
        *( sdCommand + 4 ) = ( uint8_t ) ( i & 0x000000FF );
        *( sdCommand + 5 ) = sdCRC7( sdCommand );
        for( j = 6; j < 700; j++){
            *( sdCommand + j ) = 0xFF;
        }
        sdCardSelect();
        spi_write_read_blocking( spi0, sdCommand, sdResponse, 700*sizeof( uint8_t ));
        sdCardDeselect();

        if( i == 0 ){
            for( k = 0; k < 700; k++ ){ // k < 700 && i == 0 
                printf( "SD Card 0th Sector: 0x%X %c %i\n", *( sdResponse + k ), *( sdResponse + k ), k );
            }
        }

        for( j = 0; j < 700; j++){
            if( *( sdResponse + j ) == 0xFF && *( sdResponse + j + 1 ) == 0xFF && *( sdResponse + j + 2 ) == 0xFF && *( sdResponse + j + 3 ) == 0xFE && *( sdResponse + j + 4 ) == 0xEB){
                bootSector = i;
                printf("BOOT SECTOR: 0x%X %i\n", bootSector, bootSector );
                for( k = 0; k < 700; k++ ){
                    printf( "CMD17 sdResponse: 0x%X %c %i\n", *( sdResponse + k ), *( sdResponse + k ), k );
                }
            }
        }

        if( i > 10000 ){
            break;
        }
    }
}
