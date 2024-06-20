#set(CMAKE_C_FLAGS "-g -lm")

# add the sources here
#set(sources
#    src/main.c
#    src/stm32f10x_it.c
#)
aux_source_directory(src sources)
aux_source_directory(src/driver sources)
aux_source_directory(src/protocol sources)

# GCC optimization level: use -O0 in debug build, otherwise -O2
if(CMAKE_BUILD_TYPE MATCHES Debug)
  set(opt_level -O0)
else()
  set(opt_level -Os)
endif()

# set some project constants
set(elf_file ${application_name}.elf)
set(bin_file ${application_name}.bin)
set(hex_file ${application_name}.hex)
set(map_file ${application_name}.map)
set(lss_file ${application_name}.lss)

# add sources to elf file
add_executable(${elf_file} ${sources})

target_include_directories(${elf_file} PUBLIC inc inc/driver inc/protocol)

# link libraries to project
target_link_libraries(${elf_file} PUBLIC stm32f10x core STM32F10x_FreeRTOS)

# set additional for compiler and linker: optimization and generate map file
set(additional_compiler_flags ${opt_level})
set(additional_linker_flags -Wl,-Map=${map_file},--cref,--no-warn-mismatch)
target_compile_options(${elf_file} PRIVATE ${additional_compiler_flags})
target_link_libraries(${elf_file} PRIVATE ${additional_linker_flags})

# remove unused sections
target_link_libraries(${elf_file} PUBLIC "-g -Wl,--gc-sections")

# link with linker file
target_link_libraries(${elf_file} PUBLIC -T${LINKER_SCRIPT})

target_link_options(${elf_file} PRIVATE
  LINKER:--print-memory-usage
  LINKER:-Map,${CMAKE_CURRENT_BINARY_DIR}/${elf_file}.map
)

# show size of resulting firmware image
add_custom_target(${elf_file}-size DEPENDS ${elf_file} COMMAND ${ARM_SIZE_EXECUTABLE} -B ${elf_file})

# generate extended listing
add_custom_target(${lss_file} DEPENDS ${elf_file} COMMAND ${ARM_OBJDUMP_EXECUTABLE} -S ${elf_file} > ${lss_file})

# create binary and hex files
add_custom_target(${hex_file} DEPENDS ${elf_file} COMMAND ${ARM_OBJCOPY_EXECUTABLE} -Oihex ${elf_file} ${hex_file})
add_custom_target(${bin_file} DEPENDS ${elf_file} COMMAND ${ARM_OBJCOPY_EXECUTABLE} -Obinary ${elf_file} ${bin_file})
add_custom_target(${application_name} ALL DEPENDS ${elf_file}-size ${bin_file} ${hex_file} ${lss_file})

# OpenOCD targets
add_custom_target(flash DEPENDS ${elf_file} COMMAND ${OPENOCD_EXECUTABLE}
  -f ${OPENOCD_CONFIG}
  --command "program ${elf_file} reset exit")

# serial monitor settings
add_custom_target(
  monitor
  COMMAND picocom -b 115200 --imap lfcrlf ${UART_PORT}
)

# commands to debug
add_custom_target(
  gdb
  DEPENDS ${elf_file}
  COMMAND ${OPENOCD_EXECUTABLE} -f ${OPENOCD_CONFIG} &
  COMMAND ${ARM_GDB_EXECUTABLE} -x ${CMAKE_SOURCE_DIR}/tools/gdbinit --se=${elf_file}
)
