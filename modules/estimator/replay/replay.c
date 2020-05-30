#include <stdio.h>
#include <stdint.h>

#define MAX_LOG_MSG_SIZE 2048

static uint16_t crc16_ccitt(const void *buf, size_t len, uint16_t crc) {
    for (size_t i = 0; i < len; i++) {
        crc = crc ^ (((uint8_t*)buf)[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

void process_valid_msg(uint32_t msg_size, void* msg) {

}

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("provide log file name as first argument");
        return 1;
    }

    FILE *fp;
    fp = fopen(argv[1], "r");

    fseek(fp, 0L, SEEK_END);
    const long file_size = ftell(fp);
    fseek(fp, 0L, SEEK_SET);

    uint32_t msg_size;
    uint16_t crc;
    uint8_t msg[MAX_LOG_MSG_SIZE];

    while (file_size-ftell(fp) > 6) {
        fgets(&msg_size, sizeof(msg_size), fp);
        if (msg_size > MAX_LOG_MSG_SIZE) {
            printf("invalid data: msg size > maximum message size");
            return 1;
        }

        if (msg_size+2 > file_size-ftell(fp)) {
            printf("invalid data: msg size > remaining bytes");
            return 1;
        }

        fgets(&crc, sizeof(crc), fp);
        fgets(&msg, msg_size, fp)

        uint16_t crc_computed = crc16_ccitt(msg, msg_size, 0);
        if (crc_computed != crc) {
            printf("invalid data: crc mismatch");
            return 1;
        }

        process_valid_msg(msg_size, msg);
    }
}
