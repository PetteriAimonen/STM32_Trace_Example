# STM32 ITM/ETM trace example

This is an example on how to configure the TPIU/DWT/ITM/ETM blocks
on a Cortex-M3 microcontroller for tracing the execution. Trace data is
output from the TRACESWO pin.

Designed to run especially on STM32 Value Line Discovery board, but should
be easily adaptible to other boards also. Note that the STM32F100 chip on
value line discovery does not have ETM feature.

What this does:

1. Configures the trace pin to output TPIU formatted trace from both ITM and ETM.
2. Blinks a led, while monitored by ITM tracing.
3. Causes periodic interrupts, where it runs bubblesort while tracing it with ETM.

# Trace configuration with OpenOCD

The file configure-trace.openocd contains an alternative way to configure tracing
without modifying the actual executable.

# Random notes

OpenOCD also has some built-in ETM configuration support. If anyone has an
example on how to use it, I'm interested.

Atleast STM32F407 seems to require a few extra settings: `ETM_CR` to `0xd90` to set
the trace port internal width. `ETM_TEEVR` to `0x6F` to do something related to
"start/stop resources"..
