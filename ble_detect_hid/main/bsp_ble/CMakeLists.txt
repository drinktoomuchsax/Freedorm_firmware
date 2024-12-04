idf_component_register(
    SRCS "ble_module.c"
    INCLUDE_DIRS "."
    REQUIRES bsp_button driver
)
