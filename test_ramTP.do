# Used to check if the addresses are incremented correctly

vlib work
vlog -timescale 1ns/1ns ramTP.v
vsim -L altera_mf_ver GameBoard
log {/*} 
add wave {/*}

force {Clk} 0 0, 1 1 -r 2

force {enable} 1

run 150