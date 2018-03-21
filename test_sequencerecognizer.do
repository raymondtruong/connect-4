vlib work
vlog -timescale 1ns/1ns sequence_recognizer.v
vsim sequence_recognizer

log {/*}
add wave {/*}

# clock
force {next} 0 0, 1 5 -repeat 10
force {reset} 1

# reset
force {in[1]} 0
force {in[0]} 0
run 14ns

force {in[1]} 0
force {in[0]} 1
run 50ns

force {in[1]} 0
force {in[0]} 0
run 10ns

force {in[1]} 1
force {in[0]} 0
run 50ns