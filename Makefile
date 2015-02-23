CC      = arm-none-eabi-gcc
CFLAGS  = -I . -std=gnu99
CFLAGS += -fno-common -O1
CFLAGS += -g3 -mcpu=cortex-m3 -mthumb -DSTM32F10X_MD_VL
CFLAGS += -Wall
LFLAGS  = -Tstm32.ld -nostartfiles

GDB = arm-none-eabi-gdb
OOCD = openocd
OOCDFLAGS = -f interface/stlink-v1.cfg -f target/stm32f1x_stlink.cfg

BINNAME = trace_example.elf
CSRC = trace_example.c

all: $(BINNAME)

clean:
	rm -f *.elf *.o

program: $(BINNAME)
	$(OOCD) $(OOCDFLAGS) \
	  -c "init" -c "targets" -c "reset halt" \
	  -c "flash write_image erase $<" -c "verify_image $<" \
	  -c "reset run" -c "shutdown"

debug: $(BINNAME)
	$(GDB) -iex 'target extended | $(OOCD) $(OOCDFLAGS) -c "gdb_port pipe"' \
               -iex 'mon halt' $<

$(BINNAME): $(CSRC) stm32.ld
	$(CC) $(CFLAGS) $(LFLAGS) -o $@ $<   


