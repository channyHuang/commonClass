FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

find_package(OpenSceneGraph REQUIRED)

include_directories(${projectName}
	${OSG_INCLUDE_DIR}
	"./../"
	)
	
link_directories(${projectName}
	${OSG_LIBRARY_DIR}
	)

add_library(commonOsg
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(commonOsg PRIVATE
	imgui
)