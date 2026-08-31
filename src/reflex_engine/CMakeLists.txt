# Synapto-Reflex Engine CMake Target

add_executable(synapto_reflex
    src/main.cpp
)

target_include_directories(synapto_reflex PRIVATE
    include
    ${open62541_SOURCE_DIR}/include
    ${open62541_BINARY_DIR}
)

target_link_libraries(synapto_reflex PRIVATE
    open62541
    Threads::Threads
)

if (UNIX AND NOT APPLE)
    target_link_libraries(synapto_reflex PRIVATE rt)
endif()
