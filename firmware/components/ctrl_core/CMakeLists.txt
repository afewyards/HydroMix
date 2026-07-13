file(GLOB CTRL_SRCS "*.c")
idf_component_register(
    SRCS ${CTRL_SRCS}
    INCLUDE_DIRS "include")
