#include <stdio.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "vl6180_pi.h"

int main() {
    int device = 9;  // I2C 버스 번호 설정 (예: /dev/i2c-9)
    int default_addr = VL6180_DEFAULT_ADDR;

    // VL6180X 센서 초기화
    vl6180 sensor = vl6180_initialise_address(device, default_addr);
    if (sensor < 0) {
        printf("센서 초기화 실패\n");
        return 1;
    }
    printf("센서 초기화 성공\n");

    // 거리 측정
    int distance = get_distance(sensor);
    if (distance >= 0) {
        printf("측정된 거리: %d mm\n", distance);
    }

    // 핸들 닫기
    close(sensor);

    return 0;
}
