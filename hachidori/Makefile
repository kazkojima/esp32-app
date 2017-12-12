#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := hachidori

include $(IDF_PATH)/make/project.mk

paramlist:
	grep '//@' main/*.[ch] | ./pext.py > main/paramlist
