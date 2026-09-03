set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PUBLIC FILE_SET headers TYPE HEADERS FILES
        Uart.hpp
        UartMonitor.hpp
    PRIVATE
        Uart.cpp
)

if(${PLATFORM} STREQUAL "native" OR ${PLATFORM} STREQUAL "emu")
    # native platform (Windows, MacOS, Linux)
    target_sources(${PROJECT_NAME}
        PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/native FILES
            native/coco/platform/Uart_native.hpp
            native/coco/platform/UartMonitor_native.hpp
    )
    if(WIN32)
        # Win32/IOCP/SetupAPI
        target_sources(${PROJECT_NAME}
            PUBLIC FILE_SET platform_headers FILES
                native/coco/platform/Uart_Win32.hpp
                native/coco/platform/UartMonitor_SetupAPI.hpp
            PRIVATE
                native/coco/platform/Uart_Win32.cpp
                native/coco/platform/UartMonitor_SetupAPI.cpp
        )
    elseif(LINUX)
        # io_uring
        target_sources(${PROJECT_NAME}
            PUBLIC FILE_SET platform_headers FILES
                native/coco/platform/Uart_io_uring.hpp
                native/coco/platform/UartMonitor_udev.hpp
            PRIVATE
                native/coco/platform/Uart_io_uring.cpp
                native/coco/platform/UartMonitor_udev.cpp
        )
    endif()
elseif(${PLATFORM} MATCHES "^nrf52")
    target_sources(${PROJECT_NAME}
        PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/nrf52 FILES
            nrf52/coco/platform/Uart_UARTE_TIMER.hpp
        PRIVATE
            nrf52/coco/platform/Uart_UARTE_TIMER.cpp
        )
elseif(${PLATFORM} MATCHES "^stm32f4")
    # todo
elseif(${PLATFORM} MATCHES "^stm32")
    target_sources(${PROJECT_NAME}
        PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/stm32 FILES
            stm32/coco/platform/Rs485_UART_DMA.hpp
            stm32/coco/platform/Uart_UART_DMA.hpp
        PRIVATE
            stm32/coco/platform/Rs485_UART_DMA.cpp
            stm32/coco/platform/Uart_UART_DMA.cpp
    )
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ..
)

target_link_libraries(${PROJECT_NAME}
    coco::coco
    coco-loop::coco-loop
    coco-device::coco-device
)
if(WIN32)
    target_link_libraries(${PROJECT_NAME} SetupAPI cfgmgr32)
endif()
if(LINUX)
    target_link_libraries(${PROJECT_NAME} libudev::libudev)
endif()

# install the library
install(TARGETS ${PROJECT_NAME}
    FILE_SET headers DESTINATION include/coco
    FILE_SET platform_headers DESTINATION include
)
