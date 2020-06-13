COMPONENT_OBJS += ili9488_test_main.o
COMPONENT_SRCDIRS += .

# Component to test
COMPONENT_OBJS += ../../../ili9488/ili9488.o
COMPONENT_SRCDIRS += ../../../ili9488
COMPONENT_OBJS += ../../../common/display_spi_interface.o
COMPONENT_SRCDIRS += ../../../common