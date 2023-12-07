set(sources
    src/misc.c
    #src/stm32f10x_adc.c
    #src/stm32f10x_bkp.c
    #src/stm32f10x_can.c
    #src/stm32f10x_cec.c
    #src/stm32f10x_crc.c
    #src/stm32f10x_dac.c
    #src/stm32f10x_dbgmcu.c
    #src/stm32f10x_dma.c
    #src/stm32f10x_exti.c
    src/stm32f10x_flash.c
    #src/stm32f10x_fsmc.c
    src/stm32f10x_gpio.c
    #src/stm32f10x_i2c.c
    src/stm32f10x_rcc.c
    src/stm32f10x_rtc.c
    #src/stm32f10x_sdio.c
    #src/stm32f10x_spi.c
    src/stm32f10x_tim.c
    src/stm32f10x_usart.c
    #src/stm32f10x_wwdg.c
)

add_library(stm32f10x ${sources})

# add include directories for StdPeriph library and CMSIS
target_include_directories(stm32f10x PUBLIC inc)
target_include_directories(stm32f10x PUBLIC inc/cmsis/core)
target_include_directories(stm32f10x PUBLIC inc/cmsis/device)

# set up some defines for the StdPeriph library
target_compile_definitions(stm32f10x PUBLIC USE_STDPERIPH_DRIVER)
target_compile_definitions(stm32f10x PUBLIC ${DEVICE_FAMILY})
target_compile_definitions(stm32f10x PUBLIC HSE_VALUE=${HSE_VALUE})

# additional compiler options: use size-optimized version of library in release build, use -O0 in debug build
if(CMAKE_BUILD_TYPE MATCHES Debug)
  set(additional_flags -O0)
else()
  set(additional_flags -Os)
endif()

target_compile_options(stm32f10x PRIVATE ${additional_flags})
