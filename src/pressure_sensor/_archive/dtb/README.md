# Bar10XT I2C 부트 디바이스트리 (보관용)

Jetson Orin(p3767-0003 / p3768-0000)에서 Keller Bar10XT 압력센서를 I2C로 붙이기 위해
시도했던 부트 디바이스트리(DTB) 변형들. 코드가 아니라 하드웨어 셋업 자산이므로,
재플래싱하거나 I2C 인식 문제가 재발할 때 사용한다.

| 파일 | 내용 |
|------|------|
| `dtb-original-backup.dtb` | 수정 전 순정 백업. I2C 클럭 400kHz(0x61a80) |
| `kernel_...-custom-i2c100k.dtb` | 하드웨어 I2C 클럭을 100kHz(0x186a0)로 낮춘 버전. 400kHz 불안정 시 사용 |
| `kernel_...-custom-i2cgpio.dtb` | 하드웨어 I2C 비활성화 + GPIO 비트뱅잉 소프트웨어 I2C(`i2c-gpio`, 5us delay) 추가 |

디컴파일: `dtc -I dtb -O dts -o out.dts <file>.dtb`
