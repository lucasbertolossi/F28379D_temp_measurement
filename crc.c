/* crc.c
 * computes 8-bit CRC
 * Source: https://barrgroup.com/embedded-systems/how-to/crc-calculation-c-code
 */

/*
 * The width of the CRC calculation and result.
 * Modify the typedef for a 16 or 32-bit CRC standard.
 */
#include <stdint.h>
#include <stdio.h>
#define WIDTH  (8 * sizeof(uint16_t))
#define TOPBIT (1 << (WIDTH - 1))

#define POLYNOMIAL_CRC 0x107
uint16_t  crcTable[256];


void crcInit(void)
{
    uint16_t  remainder;
    int i=0;
    int j=0;

    /*
     * Compute the remainder of each possible dividend.
     */
    for (j=0; j < 256; ++j)
    {
        /*
         * Start with the dividend followed by zeros.
         */
        remainder = j << (WIDTH - 8);

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (i = 8; i > 0; --i)
        {
            /*
             * Try to divide the current data bit.
             */
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL_CRC;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        /*
         * Store the result into the table.
         */
        crcTable[j] = remainder;
    }

}


uint16_t crcFast(uint16_t const message[], int nBytes)
{
    uint16_t data;
    uint16_t remainder = 0;
    int i=0;

    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (i=0; i < nBytes; ++i)
    {
        data = message[i] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (remainder);

}   /* crcFast() */
