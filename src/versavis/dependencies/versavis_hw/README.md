VersaVIS Board package for arduino IDE

## Add board support
* To use, add https://github.com/ethz-asl/versavis_hw/raw/master/package_VersaVIS_index.json in Arduino IDE -> File -> Preferences -> Additional Boards Manager URLs.
* Add the VersaVIS board to the board manager (Tools -> Board -> Board Manager)

## Pinout for VersaVIS 1.0 and VersaVIS 1.1
based on [this](https://docs.google.com/spreadsheets/d/1Ky8IHlCCgtuCESapGA7kTjg_rHfAfQyLWzFp4BHC_iQ/edit#gid=0)

| Port ||        Pin      ||||                                     | Periphery |
|------|--------|----------|------|-----------|---------|----------|-----------|
| Port | Number | Physical | Kind | Direction | Logical | Function |           |
| A    | 14     | 23       | D    | O         | 2       | DAUX1    | AUX       |
| A    | 04     | 9        | A    | I         | 17      | AAUX1    | AUX       |
| A    | 05     | 10       | A    | I         | 18      | AAUX2    | AUX       |
| A    | 27     | 39       | D    | O         | 26      | DAUX2    | AUX       |
| A    | 15     | 24       | D    | I         | 5       | ExpC0    | CAM0      |
| A    | 02     | 3        | D    | O         | 14      | TrigC0   | CAM0      |
| A    | 20     | 29       | D    | I         | 6       | ExpC1    | CAM1      |
| B    | 08     | 7        | D    | O         | 15      | TrigC1   | CAM1      |
| A    | 21     | 30       | D    | I         | 7       | ExpC2    | CAM2      |
| B    | 09     | 8        | D    | O         | 16      | TrigC2   | CAM2      |
| A    | 11     | 11       | D    | I         | 3       | PPS      | GPS       |
| A    | 10     | 15       | D    | I         | 4       | ExtClk   | GPS       |
| A    | 22     | 31       | D    | IO        | 20      | SDA      | I2C       |
| A    | 23     | 32       | D    | O         | 21      | SCL      | I2C       |
| A    | 30     | 45       | D    | I         | 44      | SWCLK    | PROG      |
| A    | 31     | 46       | D    | IO        | 45      | SWDIO    | PROG      |
| A    | 18     | 27       | D    | O         | 10      | SS1      | SPI1      |
| A    | 12     | 21       | D    | I         | 22      | MISO1    | SPI1      |
| B    | 10     | 19       | D    | O         | 23      | MOSI1    | SPI1      |
| B    | 11     | 20       | D    | O         | 24      | SCK1     | SPI1      |
| A    | 13     | 22       | D    | I         | 38      | DR1      | SPI1      |
| A    | 06     | 11       | D    | O         | 8       | SS2      | SPI2      |
| A    | 07     | 12       | D    | I         | 9       | DR2      | SPI2      |
| A    | 16     | 25       | D    | O         | 11      | MOSI2    | SPI2      |
| A    | 19     | 28       | D    | I         | 12      | MISO2    | SPI2      |
| A    | 17     | 26       | D    | O         | 13      | SCK2     | SPI2      |
| A    | 00     | 1        | S    |           |         | XTAL     | System    |
| A    | 01     | 2        | S    |           |         | XTAL     | System    |
| A    | 09     | 14       | D    | I         | 0       | RX1      | UART1     |
| A    | 08     | 13       | D    | O         | 1       | TX1      | UART1     |
| B    | 22     | 37       | D    | O         | 30      | TX2      | UART2     |
| B    | 23     | 38       | D    | I         | 31      | RX2      | UART2     |
| A    | 28     | 41       | D    | O         | 27      | USB_HE   | USB       |
| A    | 24     | 33       | D    | IO        | 28      | USB-     | USB       |
| A    | 25     | 34       | D    | IO        | 29      | USB+     | USB       
