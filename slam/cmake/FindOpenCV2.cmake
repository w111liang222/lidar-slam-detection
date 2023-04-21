unset(OPENCV_FOUND)
unset(OPENCV_INCLUDE_DIRS)
unset(OPENCV_LIB)

find_path(OPENCV_INCLUDE_DIRS NAMES
  opencv2/core.hpp
  HINTS
  /usr/include/opencv4
  /usr/local/include/opencv4)

foreach(LIB opencv_core opencv_highgui opencv_imgproc opencv_features2d opencv_calib3d opencv_flann opencv_imgcodecs opencv_video opencv_dnn opencv_tracking)
        set(FOUND_LIB "FOUND_LIB-NOTFOUND")
        find_library(FOUND_LIB NAMES
          ${LIB}
          HINTS
          /usr/lib
          /usr/local/lib)
        list(APPEND OPENCV_LIB ${FOUND_LIB})
        message("Found Lib: ${FOUND_LIB}")
endforeach(LIB)

if (OPENCV_INCLUDE_DIRS AND OPENCV_LIB)
  set(OPENCV_FOUND 1)
endif (OPENCV_INCLUDE_DIRS AND OPENCV_LIB)
