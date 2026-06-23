cmake_minimum_required(VERSION 4.0)

project(nativefiledialog)

FILE(GLOB LIBRARY_FILES_COMMON 
	"*.h" "*.inl"
	nfd_common.c)

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

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED IMPORTED_TARGET gtk+-3.0)
target_link_libraries(${PROJECT_NAME} PRIVATE PkgConfig::GTK3)

