vlib work
vlog -timescale 1ns/1ns main.v
vlog -timescale 1ns/1ns ramTP.v
vsim -L altera_mf_ver main

log {/*}
add wave {/*}

# clock
force {CLOCK_50} 0 0, 1 1 -repeat 2

# write to column 0
force {SW[6]} 1
force {SW[5]} 0
force {SW[4]} 0
force {SW[3]} 0
force {SW[2]} 0
force {SW[1]} 0
force {SW[0]} 0

force {KEY[0]} 1

force {KEY[0]} 0
run 10ns

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns

# write to column 1
force {SW[6]} 0
force {SW[5]} 1
force {SW[4]} 0
force {SW[3]} 0
force {SW[2]} 0
force {SW[1]} 0
force {SW[0]} 0

force {KEY[0]} 1

force {KEY[0]} 0
run 10ns

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns

force {KEY[0]} 1
run 10ns

force {KEY[0]} 0
run 10ns


# read 
force {KEY[0]} 1
run 100ns

force {address} 2#111111
run 10ns

force {address} 2#101010
run 10ns

force {address} 2#111110
run 10ns

force {address} 2#100011
run 10ns

force {address} 2#101011
run 10ns

force {address} 2#101100
run 10ns