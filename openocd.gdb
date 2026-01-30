target extended-remote localhost:3333
set print asm-demangle on
monitor arm semihosting on
load
kill
start
