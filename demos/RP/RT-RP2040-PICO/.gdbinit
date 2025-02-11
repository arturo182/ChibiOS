source /Users/vidma/flyuav/px4_rpi/PX4-Autopilot/platforms/nuttx/Debug/ARMv7M
help armv7m

set mem inaccessible-by-default off

set print pretty

set pagination off

set history save on
set history filename ~/.gdb_history
set history size unlimited
set history expansion on

set logging on


target remote localhost:3333
monitor reset halt
file build/ch.elf

set remotetimeout 20

load
#break nsh_main
c

info program

info f
i locals
info threads
i stack