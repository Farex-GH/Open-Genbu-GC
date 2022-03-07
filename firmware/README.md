BUILD INSTRUCTIONS

1. git clone the repsoitory

2. cd genbu-fw/firmware

3. git submodule update --init

4. ./setup,sh

5. mkdir build

6. cd build

7. cmake ..

8. make -j4

FIRMWARE FLASH INSTRUCTIONS

1. Plug in Raspberry Pi Pico to a PC. If reflashing, hold down the BOOTSEL button while plugging it in.

2. Copy genbu-fw.uf2 from build directory to Raspberry Pi Pico directory



DEBUGGING

1. See https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf Ch.5 and 6.

There are useful macros provided for printing via UART and GPIO toggling.
For more timing sensitve situations, use the GPIO toggling to see when certain parts of code are hit.
Otherwise if timing is not a problem, DB_PRINT works fine.


