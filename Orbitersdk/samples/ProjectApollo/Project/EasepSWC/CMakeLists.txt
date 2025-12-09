project(EasepSWC)

set(INCLUDE_DIRS
    ${NASSP_SOURCE_ROOT}/src_moon
    ${ORBITERSDK_DIR}/include
)
set(SOURCES
    ${NASSP_SOURCE_ROOT}/src_moon/EasepSWC.cpp
)

link_directories(${ORBITERSDK_DIR}/lib)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE
    Orbiter
    Orbitersdk
)