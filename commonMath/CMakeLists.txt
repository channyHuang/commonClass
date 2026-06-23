FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.c")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

project(commonMath)

include_directories(${PROJECT_NAME}
	${CMAKE_CURRENT_SOURCE_DIR}/../
)
	
add_library(${PROJECT_NAME}
	${LIBRARY_FILES_C} 
	${LIBRARY_FILES_H}
)
