This is a development of a custom Firmware for the WVC700R3 solar micro inverter because the original Firmware has many bugs and a non working MPPT.

ATTENTION: The device works with high voltage and is directly connected to the grid. Use this stuff only when you know what you do. I am not responsible for any damage or injury!

Firmware can be flashed with openocd. A binary can be found in .pio/build folder.
- Download openocd
- extract archive to disk
- copy firmware.elf into openocd.exe directory
- open command shell
- navigate to openocd directory
- run --> openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "program firmware.elf verify reset exit"

  Pinout of prog connector: (vcc) (dio) (clk) (rst) [gnd]
  (vcc is next to the controller)
