target extended-remote localhost:2331
monitor reset
monitor halt
load
monitor reset
monitor halt
info registers
x/10i $pc
break main
continue
info breakpoints
quit
