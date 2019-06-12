#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := phone

include $(IDF_PATH)/make/project.mk
$(eval $(call spiffs_create_partition_image,storage,assets,FLASH_IN_PROJECT))
