cmake_minimum_required(VERSION 4.0..6.5)

project(xatlas)
set(Lib False)

FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

if (Lib) 
    add_library(${PROJECT_NAME}
    ${LIBRARY_FILES_C} 
    ${LIBRARY_FILES_H}
    )
else()
add_executable(${PROJECT_NAME}
    ${LIBRARY_FILES_C} 
    ${LIBRARY_FILES_H}
)
endif()

link_directories(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/libs
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
    pthread
)