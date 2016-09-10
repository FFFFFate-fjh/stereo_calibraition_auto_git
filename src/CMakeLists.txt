# 增加一个可执行的二进制
#ADD_EXECUTABLE( main main.cpp )

# 增加PCL库的依赖
#FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization )

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加头文件和库文件
#ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
#INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
#LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

INCLUDE_DIRECTORIES( ${PROJECT_SOURSE_DIR}/include )

ADD_EXECUTABLE( stereo_calibration_auto stereo_calibration_auto.cpp stereo_calibration_func.cpp camera_calibrate.cpp)
TARGET_LINK_LIBRARIES( stereo_calibration_auto 
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE( test_stereo_rectify test_stereo_rectify.cpp stereo_rectify.cpp)
TARGET_LINK_LIBRARIES( test_stereo_rectify 
    ${OpenCV_LIBS} 
    )
