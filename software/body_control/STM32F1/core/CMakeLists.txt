set(sources
    src/system_stm32f10x.c
    src/startup_stm32f10x_md.s
)

add_library(core ${sources})

target_include_directories(core PUBLIC inc)
target_include_directories(core PRIVATE inc/core)

target_link_libraries(core PRIVATE stm32f10x)