# # Define a library for this submodule
set(INC_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/inc")
set(SRC_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_library(M_ShiftRegister STATIC
    ${SRC_ROOT_PATH}/shift_register.cpp
)

target_include_directories(M_ShiftRegister
    PUBLIC
        ${INC_ROOT_PATH}
)

target_link_libraries(M_ShiftRegister
    PUBLIC
        M_ShiftRegister_Cbk
)