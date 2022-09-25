cmake_minimum_required(VERSION 3.12)

project(USBJoystick C ASM)
configure_file(Ld/Link.ld Ld/Link.ld COPYONLY)
include_directories( User
    CH32V_SRC/Debug
    CH32V_SRC/Peripheral/inc
    CH32V_SRC/Core
    drivers
    rtthread/components/drivers/include
    rtthread/components/finsh
    rtthread/include
  rtthread)



add_compile_options(
  -mabi=ilp32f
  -march=rv32imafc
  -Og
  -msmall-data-limit=8
  -msave-restore
  -fmessage-length=0
  -fsigned-char
  -ffunction-sections
  -fdata-sections
  -fno-common
  -MMD
  -MP
)

add_executable(
USBJoystick
 CH32V_SRC/Debug/debug.c

 CH32V_SRC/Peripheral/src/ch32v30x_adc.c
 CH32V_SRC/Peripheral/src/ch32v30x_bkp.c
 CH32V_SRC/Peripheral/src/ch32v30x_can.c
 CH32V_SRC/Peripheral/src/ch32v30x_crc.c
 CH32V_SRC/Peripheral/src/ch32v30x_dac.c
 CH32V_SRC/Peripheral/src/ch32v30x_dbgmcu.c
 CH32V_SRC/Peripheral/src/ch32v30x_dma.c
 CH32V_SRC/Peripheral/src/ch32v30x_dvp.c
 CH32V_SRC/Peripheral/src/ch32v30x_eth.c
 CH32V_SRC/Peripheral/src/ch32v30x_exti.c
 CH32V_SRC/Peripheral/src/ch32v30x_flash.c
 CH32V_SRC/Peripheral/src/ch32v30x_fsmc.c
 CH32V_SRC/Peripheral/src/ch32v30x_gpio.c
 CH32V_SRC/Peripheral/src/ch32v30x_i2c.c
 CH32V_SRC/Peripheral/src/ch32v30x_iwdg.c
 CH32V_SRC/Peripheral/src/ch32v30x_misc.c
 CH32V_SRC/Peripheral/src/ch32v30x_opa.c
 CH32V_SRC/Peripheral/src/ch32v30x_pwr.c
 CH32V_SRC/Peripheral/src/ch32v30x_rcc.c
 CH32V_SRC/Peripheral/src/ch32v30x_rng.c
 CH32V_SRC/Peripheral/src/ch32v30x_rtc.c
 CH32V_SRC/Peripheral/src/ch32v30x_sdio.c
 CH32V_SRC/Peripheral/src/ch32v30x_spi.c
 CH32V_SRC/Peripheral/src/ch32v30x_tim.c
 CH32V_SRC/Peripheral/src/ch32v30x_usart.c
 CH32V_SRC/Peripheral/src/ch32v30x_wwdg.c



 CH32V_SRC/Core/core_riscv.c

  drivers/drv_gpio.c
  drivers/drv_usart.c

 rtthread/components/drivers/misc/pin.c
 rtthread/components/drivers/serial/serial.c
 rtthread/components/finsh/cmd.c
 rtthread/components/finsh/msh.c
 rtthread/components/finsh/msh_cmd.c
 rtthread/components/finsh/msh_file.c
 rtthread/components/finsh/shell.c
 rtthread/components/finsh/symbol.c

 rtthread/libcpu/risc-v/common/cpuport.c
 rtthread/libcpu/risc-v/common/context_gcc.S
 rtthread/libcpu/risc-v/common/interrupt_gcc.S

 rtthread/src/clock.c
 rtthread/src/components.c
 rtthread/src/cpu.c
 rtthread/src/device.c
 rtthread/src/idle.c
 rtthread/src/ipc.c
 rtthread/src/irq.c
 rtthread/src/kservice.c
 rtthread/src/mem.c
 rtthread/src/memheap.c
 rtthread/src/mempool.c
 rtthread/src/object.c
 rtthread/src/scheduler.c
 rtthread/src/slab.c
 rtthread/src/thread.c
 rtthread/src/timer.c

 rtthread/board.c

 Startup/startup_ch32v30x.S

 User/main.c
 User/ch32v30x_it.c
 User/syscalls.c
 User/system_ch32v30x.c
 User/ch32v30x_usbotg_device.c




)
add_custom_command(
 TARGET USBJoystick COMMAND riscv64-unknown-elf-objcopy ARGS -O binary USBJoystick.elf USBJoystick.bin DEPENDS USBJoystick.elf
)


target_link_options(
  USBJoystick PRIVATE
  -Og
  -mabi=ilp32f
  -march=rv32imafc
  -T Ld/Link.ld
  -nostartfiles
  --specs=nano.specs
  --specs=nosys.specs
  LINKER:--gc-sections
  LINKER:--print-memory-usage
  LINKER:-Map,${CMAKE_CURRENT_BINARY_DIR}/USBJoystick.map
)
set_target_properties(USBJoystick PROPERTIES  SUFFIX .elf)
set_target_properties(USBJoystick PROPERTIES LINK_DEPENDS Ld/Link.ld)
