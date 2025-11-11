file(GLOB_RECURSE SOURCES "src/*.c" "src/*.cpp" "TinyGPS/*.cpp")
idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS "." "inc" "TinyGPS/")
                       add_compile_options(-Wno-format -Wno-error=format)