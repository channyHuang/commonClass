FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.c")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

project(commonGeometry)

set(OUTER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../)

include_directories(${PROJECT_NAME}
	${OUTER_DIR}
	)
	
link_directories(${projectName}
)

add_library(${PROJECT_NAME}
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
)