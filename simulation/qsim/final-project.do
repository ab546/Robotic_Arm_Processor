onerror {exit -code 1}
vlib work
vlog -work work final-project.vo
vlog -work work processorTest.vwf.vt
vsim -novopt -c -t 1ps -L cycloneive_ver -L altera_ver -L altera_mf_ver -L 220model_ver -L sgate_ver -L altera_lnsim_ver work.processor_vlg_vec_tst
vcd file -direction processor.msim.vcd
vcd add -internal processor_vlg_vec_tst/*
vcd add -internal processor_vlg_vec_tst/i1/*
proc simTimestamp {} {
    echo "Simulation time: $::now ps"
    if { [string equal running [runStatus]] } {
        after 5000 simTimestamp
    }
}
after 5000 simTimestamp
run -all
quit -f





















