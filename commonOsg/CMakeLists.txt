FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

find_package(OpenSceneGraph REQUIRED osg osgViewer osgUtil osgDB osgGA osgText )

include_directories(${projectName}
	${OpenSceneGraph_INCLUDE_DIRS}
	"./../"
	)

add_library(commonOsg
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
TARGET_LINK_LIBRARIES(commonOsg PRIVATE
	imgui
	GL
	pthread
	${OpenSceneGraph_LIBRARIES}
)
else()
TARGET_LINK_LIBRARIES(commonOsg PRIVATE
	imgui
)
endif()