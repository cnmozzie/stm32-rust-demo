# rust demos for STM32

- board: [DevEBox](https://github.com/mcauser/BLACK_F407ZG)
- others: [W5500](https://doc.embedfire.com/module/module_tutorial/zh/latest/Module_Manual/spi_class/w5500.html)
- connection: GND, 3V3, SCK(PA5), CS(PA4), MISO(PA6), MOSI(PA7)

### reference

- [hello world](https://jonathanklimt.de/electronics/programming/embedded-rust/rust-on-stm32-2/)
- [HAL DEMO](https://github.com/stm32-rs/stm32f4xx-hal/tree/master)
- [booster](https://github.com/quartiq/booster)

### build the project

`cargo build --release`

### load the program

`cargo flash --chip stm32f407ZG --release`
