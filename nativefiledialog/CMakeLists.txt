cmake_minimum_required(VERSION 3.10)

project(nativefiledialog)

FILE(GLOB LIBRARY_FILES_COMMON 
	"*.h" "*.inl"
	nfd_common.c)

include_directories(${projectName}
	"./../"
	)
	
link_directories(${projectName}
	)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
FILE(GLOB LIBRARY_FILES
	nfd_gtk.c)
else()
FILE(GLOB LIBRARY_FILES 
	nfd_win.cpp)
endif()

add_library(${PROJECT_NAME}
${LIBRARY_FILES_COMMON} 
${LIBRARY_FILES}
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
	find_package(GTK2 REQUIRED)

	include_directories(${PROJECT_NAME}
		${GTK2_INCLUDE_DIRS}
	)
	target_link_libraries(${PROJECT_NAME}
		${GTK2_LIBRARIES}
	)
else()
TARGET_LINK_LIBRARIES(${PROJECT_NAME}

)
endif()
