cmake_minimum_required(VERSION 3.12)
project(Sofa.RigidBodyDynamics VERSION 25.06.99 LANGUAGES CXX)

sofa_find_package(Sofa.Simulation.Core REQUIRED)
sofa_find_package(Sofa.Component.StateContainer REQUIRED)
sofa_find_package(Sofa.Component.Mapping.NonLinear REQUIRED)
sofa_find_package(Sofa.Component.Mass REQUIRED)
sofa_find_package(Sofa.Component.Collision.Geometry REQUIRED)
sofa_find_package(Sofa.Component.Constraint.Lagrangian.Model REQUIRED)
sofa_find_package(Sofa.Component.Constraint.Projective REQUIRED)
sofa_find_package(Sofa.Component.IO.Mesh REQUIRED)
sofa_find_package(Sofa.GL.Component.Rendering3D REQUIRED)

sofa_find_package(SoftRobots REQUIRED)
sofa_find_package(SoftRobots.Inverse REQUIRED)

# List all files
set(SOFA_RBD_SRC_DIR src/sofa/RigidBodyDynamics)
set(HEADER_FILES
    ${SOFA_RBD_SRC_DIR}/config.h.in
    ${SOFA_RBD_SRC_DIR}/BaseShapesConversions.h
    ${SOFA_RBD_SRC_DIR}/Conversions.h
    ${SOFA_RBD_SRC_DIR}/GeometryConversions.h
    ${SOFA_RBD_SRC_DIR}/KinematicChainMapping.h
    ${SOFA_RBD_SRC_DIR}/KinematicChainMapping.inl
    ${SOFA_RBD_SRC_DIR}/Types.h
    ${SOFA_RBD_SRC_DIR}/URDFModelLoader.h
    )
set(HEADER_FILES_TO_MOC
    )
set(SOURCE_FILES
    ${SOFA_RBD_SRC_DIR}/init.cpp
    ${SOFA_RBD_SRC_DIR}/BaseShapesConversions.cpp
    ${SOFA_RBD_SRC_DIR}/GeometryConversions.cpp
    ${SOFA_RBD_SRC_DIR}/KinematicChainMapping.cpp
    ${SOFA_RBD_SRC_DIR}/Types.cpp
    ${SOFA_RBD_SRC_DIR}/URDFModelLoader.cpp
    )
set(README_FILES
    README.md
    )

find_package(pinocchio 3.0 REQUIRED)
find_package(coal 3.0 REQUIRED)
find_package(assimp REQUIRED)

if(WIN32)
    add_definitions(-DBOOST_ALL_NO_LIB) # Linking to boost_XXX instead of libboost_XXX
endif(WIN32)

set(Boost_USE_STATIC_LIBS OFF) #WHY ?!
find_package(Boost COMPONENTS serialization REQUIRED)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

target_link_libraries(${PROJECT_NAME} 
  Sofa.Simulation.Core
  Sofa.Component.StateContainer
  Sofa.Component.Mapping.NonLinear
  Sofa.Component.Mass
  Sofa.Component.Collision.Geometry
  Sofa.Component.Constraint.Projective
  Sofa.Component.Constraint.Lagrangian.Model
  Sofa.Component.IO.Mesh
  Sofa.GL.Component.Rendering3D
  pinocchio::pinocchio
  coal::coal
  SoftRobots
  SoftRobots.Inverse
  )

#  if(Sofa.GL_FOUND)
#      target_link_libraries(${PROJECT_NAME} Sofa.GL)
#  endif()


sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )

function(collect_dependency_dlls output_var)
    foreach(target IN LISTS ARGN)
        get_target_property(tmp_dll ${target} IMPORTED_LOCATION_RELEASE)
        if (tmp_dll)
            list(APPEND ${output_var} ${tmp_dll})
            set(${output_var} "${${output_var}}" PARENT_SCOPE)
        endif()
    endforeach()
endfunction()

set(DEPENDENCIES_dlls "")
# Boost is giving .lib instead of .dll
get_target_property(boost_serialib Boost::serialization IMPORTED_LOCATION_RELEASE)
get_filename_component(boost_lib_dir "${boost_serialib}" DIRECTORY)
get_filename_component(boost_lib_name "${boost_serialib}" NAME_WE)
list(APPEND DEPENDENCIES_dlls "${boost_lib_dir}/${boost_lib_name}.dll")
get_target_property(boost_serialib Boost::filesystem IMPORTED_LOCATION_RELEASE)
get_filename_component(boost_lib_dir "${boost_serialib}" DIRECTORY)
get_filename_component(boost_lib_name "${boost_serialib}" NAME_WE)
list(APPEND DEPENDENCIES_dlls "${boost_lib_dir}/${boost_lib_name}.dll")
collect_dependency_dlls(DEPENDENCIES_dlls
    assimp::assimp
    coal::coal
    console_bridge::console_bridge
    eigenpy::eigenpy
    octomap
    octomath
    pinocchio::pinocchio_default
    pinocchio::pinocchio_parsers
    pinocchio::pinocchio_collision
    pinocchio::pinocchio_python_parser
    pinocchio::pinocchio_visualizers
    urdfdom::urdfdom_model
    urdfdom::urdfdom_model_state
    urdfdom::urdfdom_sensor
    urdfdom::urdfdom_world
)
message(STATUS "RigidBodyDynamics dependencies DLLs: ${DEPENDENCIES_dlls}")

foreach(dll ${DEPENDENCIES_dlls})
    add_custom_command(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${dll}"
                $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMENT "Copying ${dll} to build output folder"
    )
endforeach()
install(FILES ${DEPENDENCIES_dlls} DESTINATION bin)

# Tests
# If SOFA_BUILD_TESTS exists and is OFF, then these tests will be auto-disabled
cmake_dependent_option(SOFA_RIGIDBODYDYNAMICS_BUILD_TESTS "Compile the automatic tests" OFF "SOFA_BUILD_TESTS OR NOT DEFINED SOFA_BUILD_TESTS" OFF)
if(SOFA_RIGIDBODYDYNAMICS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
