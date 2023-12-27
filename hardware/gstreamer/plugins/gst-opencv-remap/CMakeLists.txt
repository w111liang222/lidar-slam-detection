set(SOURCES
    gstopencvremap.cpp
)

set(HEADERS
    gstopencvremap.h
)

set(libname gstopencvremap)

add_library (${libname} MODULE
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
    ${OPENCV_LIB}
)