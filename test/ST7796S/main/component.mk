COMPONENT_OBJS += st7796s_test_main.o
COMPONENT_SRCDIRS += .

# Component to test
COMPONENT_OBJS += ../../../st7796s/st7796s.o
COMPONENT_SRCDIRS += ../../../st7796s
COMPONENT_OBJS += ../../../common/display_spi_interface.o
COMPONENT_SRCDIRS += ../../../common