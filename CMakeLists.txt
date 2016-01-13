project(iCubHeadMovement)


FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

# Include include_directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

# Add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

add_executable(iCubHeadMovement main.cpp)

install(TARGETS iCubHeadMovement RUNTIME DESTINATION bin)

#adding YARP and icub libraries to project
TARGET_LINK_LIBRARIES( iCubHeadMovement ${YARP_LIBRARIES})