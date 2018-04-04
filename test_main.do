vlib work
vlog -timescale 1ns/1ns main_no_vga.v
vlog -timescale 1ns/1ns keyboard_press_driver.v
vlog -timescale 1ns/1ns keyboard_inner_driver.v
vlog -timescale 1ns/1ns turn_tracker.v
vlog -timescale 1ns/1ns sequence_recognizer.v
vlog -timescale 1ns/1ns decoder.v
vlog -timescale 1ns/1ns score_system.v
vsim main_no_vga

log {/*}
add wave {/*}

# clock
force {CLOCK_50} 0 0, 1 1 -repeat 2


# simulate win

force {valid} 1
force {makeBreak} 0
run 5000ns


force {outCode} 16#1A
run 10ns

force {makeBreak} 1
run 500ns

force {makeBreak} 0
run 500ns


force {outCode} 16#22
run 10ns

force {makeBreak} 1
run 500ns

force {makeBreak} 0
run 500ns

force {outCode} 16#1A
run 10ns

force {makeBreak} 1
run 500ns

force {makeBreak} 0
run 500ns


force {outCode} 16#22
run 10ns

force {makeBreak} 1
run 500ns

force {makeBreak} 0
run 500ns

force {outCode} 16#1A
run 10ns

force {makeBreak} 1
run 500ns

force {makeBreak} 0
run 500ns


force {outCode} 16#22
run 10ns

force {makeBreak} 1
run 500ns

force {makeBreak} 0
run 500ns

force {outCode} 16#1A
run 10ns

force {makeBreak} 1
run 500ns

force {makeBreak} 0
run 500ns





run 5000ns