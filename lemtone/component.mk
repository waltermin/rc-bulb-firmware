# Legacy make component config (see CMakeLists.txt). Only the top-level .c files are
# built; test/ and web/ are not component sources.
COMPONENT_ADD_INCLUDEDIRS := .
COMPONENT_SRCDIRS := .
CFLAGS += -O2
