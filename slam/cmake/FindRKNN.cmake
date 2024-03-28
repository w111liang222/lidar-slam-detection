unset(RKNN_FOUND)
unset(RKNN_INCLUDE_DIRS)
unset(RKNN_LIBRARIES)

find_path(RKNN_INCLUDE_DIRS NAMES
  rknn_api.h
  HINTS
  /usr/include
  /usr/local/include)

foreach(LIB rknnrt)
        set(FOUND_LIB "FOUND_LIB-NOTFOUND")
        find_library(FOUND_LIB NAMES
          ${LIB}
          HINTS
          /usr/lib
          /usr/local/lib)
        list(APPEND RKNN_LIBRARIES ${FOUND_LIB})
        message("Found Lib: ${FOUND_LIB}")
endforeach(LIB)

if (RKNN_INCLUDE_DIRS AND RKNN_LIBRARIES)
  set(RKNN_FOUND 1)
else()
  set(RKNN_FOUND 0)
  set(RKNN_INCLUDE_DIRS "")
  set(RKNN_LIBRARIES "")
endif (RKNN_INCLUDE_DIRS AND RKNN_LIBRARIES)
