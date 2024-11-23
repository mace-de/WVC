This is a development of a custom Firmware for the WVC700R3 solar micro inverter because the original Firmware has many bugs and a non working MPPT.
The Firmware has an option to perfom an automatic test procedure for the mains relay. To use it, resistor R88 has to be changed to 10k ohm.
Some more Information (in german) can be found here https://www.photovoltaikforum.com/thread/209046-wvc700r3-mppt-regler-tauschen/?pageNo=11 starting at post No.110

## :exclamation: ATTENTION: The device works with high voltage and is directly connected to the grid. Use this stuff only when you know what you do. I am not responsible for any damage or injury!

Firmware can be flashed with openocd. A binary can be found in .pio/build folder. A St-Link/V2 flash adapter is needed (can be found on Amazon).
- Download openocd
- extract archive to disk
- copy firmware.elf into openocd.exe directory
- open command shell
- navigate to openocd directory
- run --> openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "program firmware.elf verify reset exit"

  Pinout of prog connector: (vcc) (dio) (clk) (rst) [gnd]
  (vcc is next to the controller)

If RDP fuse has to be unlocked run --> openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "init" -c"reset halt" -c"stm32f1x unlock 0" -c"reset halt" -c"exit"
