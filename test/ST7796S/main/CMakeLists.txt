set(srcs  
    "st7796s_test_main.c"
    "../../../st7796s/st7796s.c"
    "../../../common/display_spi_interface.c")

set(include_dirs
    "../../../include")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${include_dirs}")
