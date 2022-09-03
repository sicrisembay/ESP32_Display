set(srcs  
    "ili9488_test_main.c"
    "../../../ili9488/ili9488.c"
    "../../../common/display_spi_interface.c")

set(include_dirs
    "../../../include")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${include_dirs}")
