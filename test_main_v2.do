vlib work
vlog -timescale 1ns/1ns main.v
vlog -timescale 1ns/1ns turn_tracker.v
vlog -timescale 1ns/1ns sequence_recognizer.v
vsim main

log {/*}
add wave {/*}

# clock
force {CLOCK_50} 0 0, 1 1 -repeat 2

# simulate win

force {SW[6]} 1
force {SW[5]} 0
force {SW[4]} 0
force {SW[3]} 0
force {SW[2]} 0
force {SW[1]} 0
force {SW[0]} 0

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns

force {SW[6]} 0
force {SW[5]} 1
force {SW[4]} 0
force {SW[3]} 0
force {SW[2]} 0
force {SW[1]} 0
force {SW[0]} 0

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns

force {SW[6]} 1
force {SW[5]} 0
force {SW[4]} 0
force {SW[3]} 0
force {SW[2]} 0
force {SW[1]} 0
force {SW[0]} 0

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns

force {SW[6]} 0
force {SW[5]} 1
force {SW[4]} 0
force {SW[3]} 0
force {SW[2]} 0
force {SW[1]} 0
force {SW[0]} 0

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns


force {SW[6]} 1
force {SW[5]} 0
force {SW[4]} 0
force {SW[3]} 0
force {SW[2]} 0
force {SW[1]} 0
force {SW[0]} 0

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns

force {SW[6]} 0
force {SW[5]} 1
force {SW[4]} 0
force {SW[3]} 0
force {SW[2]} 0
force {SW[1]} 0
force {SW[0]} 0

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns

force {SW[6]} 1
force {SW[5]} 0
force {SW[4]} 0
force {SW[3]} 0
force {SW[2]} 0
force {SW[1]} 0
force {SW[0]} 0

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns

force {SW[6]} 0
force {SW[5]} 1
force {SW[4]} 0
force {SW[3]} 0
force {SW[2]} 0
force {SW[1]} 0
force {SW[0]} 0

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns

force {KEY[0]} 1

run 5000ns
