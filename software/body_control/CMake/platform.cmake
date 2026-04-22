message("-- Cmake running on platform: " ${CMAKE_HOST_SYSTEM_NAME})

if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin")
    # for MacOS X or iOS, watchOS, tvOS (since 3.10.3)
    set(UART_PORT "/dev/tty.usbserial")
elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
    # for Linux, BSD, Solaris, Minix
    set(UART_PORT "/dev/ttyUSB0")
elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
    # for Windows operating system  
    set(UART_PORT "COM4")
endif()