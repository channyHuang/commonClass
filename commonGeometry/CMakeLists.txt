CMAKE_MINIMUM_REQUIRED(VERSION 4.0)

project(commonGeometry)

find_package(GDAL REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../commonMath commonMath)

FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.c")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

include_directories(${PROJECT_NAME}
	${CMAKE_CURRENT_SOURCE_DIR}/../
)

add_library(${PROJECT_NAME}
	${LIBRARY_FILES_C} 
	${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
	GDAL::GDAL

	commonMath
)