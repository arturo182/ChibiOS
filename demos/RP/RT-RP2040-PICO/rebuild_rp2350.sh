export PATH="/usr/local/bin/picotool:$PATH"
export PICO_SDK_PATH="/Users/vidma/flyuav/px4_rpi/pico-sdk"

rm -Rf .dep build
make && picotool uf2 convert --verbose build/ch.elf build/ch.uf2 --abs-block
picotool load build/ch.uf2 && picotool reboot


arm-none-eabi-gdb -ix=.gdbinit