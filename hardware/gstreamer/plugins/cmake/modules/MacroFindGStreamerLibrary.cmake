# - macro find_gstreamer_library
#
# Copyright (c) 2010, Collabora Ltd.
#   @author George Kiagiadakis <george.kiagiadakis@collabora.co.uk>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

macro(find_gstreamer_library _name _header _abi_version)
    string(TOLOWER ${_name} _lower_name)
    string(TOUPPER ${_name} _upper_name)

    if (GSTREAMER_${_upper_name}_LIBRARY AND GSTREAMER_${_upper_name}_INCLUDE_DIR)
        set(_GSTREAMER_${_upper_name}_QUIET TRUE)
    else()
        set(_GSTREAMER_${_upper_name}_QUIET FALSE)
    endif()

    if (PKG_CONFIG_FOUND)
        pkg_check_modules(PKG_GSTREAMER_${_upper_name} gstreamer-${_lower_name}-${_abi_version})
    endif()

    if (NOT GSTREAMER_ROOT)
        if (CMAKE_SIZEOF_VOID_P MATCHES "8")
            set(GSTREAMER_ROOT $ENV{GSTREAMER_1_0_ROOT_X86_64})
        else ()
            set(GSTREAMER_ROOT $ENV{GSTREAMER_1_0_ROOT_X86})
        endif ()
    endif ()
    
    find_library(GSTREAMER_${_upper_name}_LIBRARY
                 NAMES gst${_lower_name}-${_abi_version}
                 HINTS ${PKG_GSTREAMER_${_upper_name}_LIBRARY_DIRS}
                       ${PKG_GSTREAMER_${_upper_name}_LIBDIR}
                       ${GSTREAMER_ROOT}/lib
    )

    find_path(GSTREAMER_${_upper_name}_INCLUDE_DIR
              gst/${_lower_name}/${_header}
              HINTS ${PKG_GSTREAMER_${_upper_name}_INCLUDE_DIRS}
                    ${PKG_GSTREAMER_${_upper_name}_INCLUDEDIR}
                    ${GSTREAMER_ROOT}/include
              PATH_SUFFIXES gstreamer-${_abi_version}
    )

    if (GSTREAMER_${_upper_name}_LIBRARY AND GSTREAMER_${_upper_name}_INCLUDE_DIR)
        set(GSTREAMER_${_upper_name}_LIBRARY_FOUND TRUE)
    else()
        set(GSTREAMER_${_upper_name}_LIBRARY_FOUND FALSE)
    endif()

    if (NOT _GSTREAMER_${_upper_name}_QUIET)
        if (GSTREAMER_${_upper_name}_LIBRARY)
            message(STATUS "Found GSTREAMER_${_upper_name}_LIBRARY: ${GSTREAMER_${_upper_name}_LIBRARY}")
        else()
            message(STATUS "Could NOT find GSTREAMER_${_upper_name}_LIBRARY")
        endif()

        if (GSTREAMER_${_upper_name}_INCLUDE_DIR)
            message(STATUS "Found GSTREAMER_${_upper_name}_INCLUDE_DIR: ${GSTREAMER_${_upper_name}_INCLUDE_DIR}")
        else()
            message(STATUS "Could NOT find GSTREAMER_${_upper_name}_INCLUDE_DIR")
        endif()
    endif()

    mark_as_advanced(GSTREAMER_${_upper_name}_LIBRARY GSTREAMER_${_upper_name}_INCLUDE_DIR)
endmacro()
