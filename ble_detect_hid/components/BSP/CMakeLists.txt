idf_component_register(
    SRCS    "button.c"
            "../MultiButton/multi_button.c"  # 添加外部库源文件
    INCLUDE_DIRS    "."
                     "../MultiButton"

    REQUIRES driver
)
