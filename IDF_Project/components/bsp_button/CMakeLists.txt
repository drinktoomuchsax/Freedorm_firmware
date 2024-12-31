idf_component_register(
    SRCS "button.c"
    INCLUDE_DIRS "."
    REQUIRES    MultiButton 
                driver 
                bsp_ble 
                ws2812b
                main
                lock_control
)

# 因为链接顺序的问题，component之间调用的函数符号会被链接器优化，所以需要在这里添加依赖
# 或者也可以在main里面调用一下lock_control的函数，但是这样不可维护，并且距离lock_control的功能更远，还是在这里解决优化问题
target_link_libraries(${COMPONENT_LIB} INTERFACE "${CMAKE_BINARY_DIR}/esp-idf/lock_control/liblock_control.a")
