target extended-remote /dev/cu.usbmodemDDCEC9EC1

# print demangled symbols
set print asm-demangle on

# set backtrace limit to not have infinite backtrace loops
set backtrace limit 32

monitor swdp_scan
att 1

monitor rtt

load
# start the process but immediately halt the processor
b main
continue
