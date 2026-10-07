/**
 * Cross-check the FPGA frame format against the STM32 firmware's parser.
 *
 * Reads frames captured from tb_spi_frame_tx (+dump -> frames.hex) and runs
 * them through NyanKeysFrameValid() from the firmware's nyan_keys_frame.h:
 *  - every frame the FPGA produced must validate
 *  - every single-bit corruption of it must be rejected
 *
 * Built and run by `make crosscheck FW=<path to nyan-keys-stm32-firmware>`.
 */
#include <stdio.h>
#include <string.h>

#include "nyan_keys_frame.h"

int main(int argc, char **argv)
{
    FILE   *fp;
    char    line[64];
    uint8_t frame[NYAN_KEYS_FRAME_BYTES];
    int     frames = 0, errors = 0;

    if (argc != 2 || (fp = fopen(argv[1], "r")) == NULL) {
        fprintf(stderr, "usage: %s frames.hex\n", argv[0]);
        return 2;
    }

    if (NyanKeysCrc8((const uint8_t *)"123456789", 9) != 0xF4) {
        printf("ERROR firmware crc8 check value\n");
        errors++;
    }

    while (fgets(line, sizeof(line), fp)) {
        if (strlen(line) < 2 * NYAN_KEYS_FRAME_BYTES)
            continue;
        for (int i = 0; i < NYAN_KEYS_FRAME_BYTES; i++) {
            unsigned b;
            sscanf(&line[2 * i], "%2x", &b);
            frame[i] = (uint8_t)b;
        }
        frames++;
        if (!NyanKeysFrameValid(frame)) {
            printf("ERROR frame %d rejected by firmware: %s", frames, line);
            errors++;
        }
        for (int bit = 0; bit < 8 * NYAN_KEYS_FRAME_BYTES; bit++) {
            frame[bit / 8] ^= (uint8_t)(1u << (bit % 8));
            if (NyanKeysFrameValid(frame)) {
                printf("ERROR frame %d accepted with bit %d flipped\n", frames, bit);
                errors++;
            }
            frame[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        }
    }
    fclose(fp);

    if (frames == 0) {
        printf("ERROR no frames read\n");
        errors++;
    }
    if (errors == 0) printf("PASS crosscheck: %d FPGA frames valid in firmware, all single-bit errors caught\n", frames);
    else             printf("FAIL crosscheck: %d error(s)\n", errors);
    return errors ? 1 : 0;
}
