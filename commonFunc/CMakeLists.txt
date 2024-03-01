FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.c")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

set(projectName commonFunc)

include_directories(${projectName}
	"./../"
	)
	
link_directories(${projectName}
	)

add_library(${projectName}
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${projectName}
)