set(SOURCES
    gstflirsrc.cpp
)

set(HEADERS
    gstflirsrc.h
)

set(libname gstflirsrc)

add_library(${libname} MODULE
    ${SOURCES}
    ${HEADERS}
)

add_definitions(-std=c++11 -Wno-deprecated-declarations -Wno-write-strings)

target_link_libraries (${libname} LINK_PUBLIC
    ${GLIB2_LIBRARIES}
    ${GOBJECT_LIBRARIES}
    ${GSTREAMER_LIBRARY}
    ${GSTREAMER_BASE_LIBRARY}
    ${GSTREAMER_VIDEO_LIBRARY}
    flir_driver
)
