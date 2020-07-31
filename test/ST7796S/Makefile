#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := test_st7796s

# Get Path of ESP32_Display
ESP32_DISPLAY_PATH := $(dir $(CURDIR))
ESP32_DISPLAY_PATH := $(dir $(ESP32_DISPLAY_PATH:%/=%))
$(info ESP32_DISPLAY_PATH is $(ESP32_DISPLAY_PATH))
export ESP32_DISPLAY_PATH

ifndef IDF_PATH
$(error IDF_PATH variable is not set to a valid directory.)
endif

include $(IDF_PATH)/make/project.mk