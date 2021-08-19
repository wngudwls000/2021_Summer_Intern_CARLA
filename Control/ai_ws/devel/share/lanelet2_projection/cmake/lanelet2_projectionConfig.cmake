# File was automatically generated by mrt_cmake_modules
#
# This file exports the following variables:
# lanelet2_projection_LIBRARIES: Library targets that should be linked against
# lanelet2_projection_EXPORTED_TARGETS: As required by catkin
# lanelet2_projection_EXPORTS_TARGETS: To indicate this package exports targets instead of plain libraries


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was packageConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

####################################################################################

set(lanelet2_projection_LIBRARIES lanelet2_projection::lanelet2_projection;lanelet2_projection::lanelet2_projection_compiler_flags)  # things that dependend packages should link against

# catkin wants this. Don't bother with it.
set(lanelet2_projection_EXPORTED_TARGETS)

# mark this as a catkin project
set(lanelet2_projection_FOUND_CATKIN_PROJECT TRUE)
set(lanelet2_projection_EXPORTS_TARGETS TRUE)

macro(_init_package_dependencies)
    # gets all targets that this package export_depends on and sets var to the name of these targets.
    # because this config file might be used recursively we have to be extra careful that variables set here do not affect each other
    if(TARGET lanelet2_projection::auto_deps_export)
        return()
    endif()
    if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/auto_dep_vars.cmake")
        message(FATAL_ERROR "Project lanelet2_projection did not export its dependencies properly! No auto_dep file was found.")
    endif()
    # Resolve deps in the same style they were resolved when this project was built
    include("${CMAKE_CURRENT_LIST_DIR}/auto_dep_vars.cmake") # sets _lanelet2_projection_EXPORT_PACKAGES_

    set(lanelet2_projectionAutoDeps_PREFIX "lanelet2_projection")
    set(lanelet2_projectionAutoDeps_NO_CATKIN_EXPORT TRUE) # disable variable export because catkin is not listening here
    if(NOT mrt_cmake_modules_FOUND)
        # We need the mrt_cmake_modules because they provide extra CMake Modules for finding thirdparty stuff
        find_package(mrt_cmake_modules REQUIRED)
    endif()
    find_package(lanelet2_projectionAutoDeps REQUIRED
        COMPONENTS ${_lanelet2_projection_EXPORT_PACKAGES_}
        PATHS ${CMAKE_CURRENT_LIST_DIR} NO_DEFAULT_PATH
        )
    if(NOT OpenMP_FOUND)
        find_package(OpenMP) # openmp is by default always a dependency
    endif()
    unset(lanelet2_projectionAutoDeps_PREFIX)
    unset(lanelet2_projectionAutoDeps_NO_CATKIN_EXPORT)
endmacro()

if(lanelet2_projection_LIBRARIES)
    _init_package_dependencies()
endif()

# add the targets
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/lanelet2_projectionTargets.cmake")
    include("${CMAKE_CURRENT_LIST_DIR}/lanelet2_projectionTargets.cmake")
endif()

# add dependencies to other things generated by this package
set(lanelet2_projection_targets )
if(lanelet2_projection_targets AND NOT TARGET lanelet2_projection::mrt_exported_targets)
    set(namespaced_exported_targets)
    foreach(t ${lanelet2_projection_targets})
        if(NOT TARGET ${target})
            add_library(lanelet2_projection::${t} INTERFACE IMPORTED)
            list(APPEND namespaced_exported_targets lanelet2_projection::${t})
        else()
            list(APPEND namespaced_exported_targets ${t})
        endif()
    endforeach()
    add_library(lanelet2_projection::mrt_exported_targets INTERFACE IMPORTED)
    add_dependencies(lanelet2_projection::mrt_exported_targets ${namespaced_exported_targets})
    list(APPEND lanelet2_projection_LIBRARIES lanelet2_projection::mrt_exported_targets)
    unset(namespaced_exported_targets)
endif()

set(lanelet2_projection_EXTRAS_FILE )
if(lanelet2_projection_EXTRAS_FILE)
    include(${CMAKE_CURRENT_LIST_DIR}/${lanelet2_projection_EXTRAS_FILE})
endif()
unset(lanelet2_projection_EXTRAS_FILE)
unset(lanelet2_projection_targets)
