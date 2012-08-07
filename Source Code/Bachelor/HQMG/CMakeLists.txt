CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(HQMG)


SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/build/)
FIND_PACKAGE(Niven REQUIRED COMPONENTS Core)


# niven is built with RTTI disabled (/GR-) and unicode-aware
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR- /D_UNICODE")

#include headers and source files
SET (HEADERS
	ShardFileParser.h
	MeshExtractor.h
	GuidanceField.h
	MultivariatePolynom.h
	Atom.h
	SimpleParticle.h
	Spline.h
	Surface.h
	Octree.h
	Front.h
	Vertex.h)
SET (SOURCES
	ShardFileParser.cpp
	HighQualityMeshGeneration2D.cpp
	MeshExtractor.cpp
	GuidanceField.cpp
	MultivariatePolynom.cpp	
	Atom.cpp
	SimpleParticle.cpp
	Spline.cpp
	Surface.cpp
	Octree.cpp
	Front.cpp
	Vertex.cpp)

# Must link both against niven and Boost
INCLUDE_DIRECTORIES(${NIVEN_INCLUDE_DIR} ${Boost_INCLUDE_DIR})
ADD_EXECUTABLE(HQMG ${HEADERS} ${SOURCES})


# The test application uses niven core, so link against it
TARGET_LINK_LIBRARIES(HQMG ${NIVEN_CORE_LIBRARY} ${NIVEN_ENGINE_LIBRARY} ${NIVEN_RENDER_LIBRARY} ${NIVEN_VOLUME_LIBRARY})


# Adds a new build target which copies the niven binaries to
# the output folder
NIVEN_COPY_BINARIES(${CMAKE_BINARY_DIR})