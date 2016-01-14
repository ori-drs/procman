# Macros for automatically compiling LCM types into C, Java, and Python
# libraries.
#
# The primary macro is:
#     lcmtypes_build(TYPES type1.lcm [type2.lcm ...])
#
# It expects that the directory ${PROJECT_SOURCE_DIR}/lcmtypes contains all
# the LCM types used by the system.  The macro generates C, Java, and Python
# bindings.  See the C, Java, and Python sections below for information on
# language specific options and generated results.
#
# After invoking this macro, the following variables will be set:
#
#   LCMTYPES_LIBS
#   LCMTYPES_JAR
#
#
# C
# ==
#
# C bindings will be placed in ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/c.
#
# The autogenerated C bindings also get compiled to a static and shared
# library.  The library prefix will be stored in LCMTYPES_LIBS on output.
# This prefix can be manually set using the C_LIBNAME option.
#
# C++
# ==
#
# C++ bindings will be placed in ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/cpp.
#
# The autogenerated CPP bindings are header only, so no library is created.
#
# Java
# ====
#
# If Java is available, then Java bindings are be generated and placed in
#    ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/java
#
# Additionally, targets are added to automatically compile the .java files to a
# .jar file. The location of this jar file is stored in LCMTYPES_JAR
#
# and the .jar file will be installed to
#   ${CMAKE_INSTALL_PREFIX}/share/java
#
#
# Python
# ======
#
# If Python is enabled, then python bindings will be generated and placed in
#    ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/python
#
# Additionally, the .py files will be installed to
#   ${CMAKE_INSTALL_PREFIX}/lib/python{X.Y}/dist-packages
#
# where {X.Y} refers to the python version used to build the .py files.
#
# ----

cmake_minimum_required(VERSION 2.6.0)

# Policy settings to prevent warnings on 2.6 but ensure proper operation on
# 2.4.
if(COMMAND cmake_policy)
    # Logical target names must be globally unique.
    cmake_policy(SET CMP0002 OLD)
    # Libraries linked via full path no longer produce linker search paths.
    cmake_policy(SET CMP0003 OLD)
    # Preprocessor definition values are now escaped automatically.
    cmake_policy(SET CMP0005 OLD)
    if(POLICY CMP0011)
        # Included scripts do automatic cmake_policy PUSH and POP.
        cmake_policy(SET CMP0011 OLD)
    endif(POLICY CMP0011)
endif()

function(lcmgen)
    execute_process(COMMAND ${LCM_GEN_EXECUTABLE} ${ARGV} RESULT_VARIABLE lcmgen_result)
    if(NOT lcmgen_result EQUAL 0)
        message(FATAL_ERROR "lcm-gen failed")
    endif()
endfunction()

function(lcmtypes_add_clean_dir clean_dir)
    get_directory_property(acfiles ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND acfiles ${clean_dir})
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${acfiles}")
endfunction()

function(lcmtypes_build_c)
    set(_lcmtypes ${ARGV})

    # set some defaults

    # library name
    set(libname "${PROJECT_NAME}_lcmtypes")

    # allow defaults to be overriden by function parameters
    set(modewords C_LIBNAME)
    set(curmode "")
    foreach(word ${ARGV})
        list(FIND modewords ${word} mode_index)
        if(${mode_index} GREATER -1)
            set(curmode ${word})
        elseif(curmode STREQUAL C_LIBNAME)
            set(libname "${word}")
            set(curmode "")
        endif()
    endforeach()

    # generate C bindings for LCM types
    set(_lcmtypes_c_dir ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/c/lcmtypes)

    # blow away any existing auto-generated files.
    file(REMOVE_RECURSE ${_lcmtypes_c_dir})

    # run lcm-gen now
    execute_process(COMMAND mkdir -p ${_lcmtypes_c_dir})
    set(lcmgen_c_flags --lazy -c --c-cpath ${_lcmtypes_c_dir} --c-hpath ${_lcmtypes_c_dir} --cinclude lcmtypes ${_lcmtypes})
    lcmgen(${lcmgen_c_flags})

    # get a list of all generated .c and .h files
    file(GLOB _lcmtypes_c_files ${_lcmtypes_c_dir}/*.c)
    file(GLOB _lcmtypes_h_files ${_lcmtypes_c_dir}/*.h)

    # run lcm-gen at compile time
    add_custom_command(OUTPUT ${_lcmtypes_c_files} ${_lcmtypes_h_files}
        COMMAND sh -c '[ -d ${_lcmtypes_c_dir} ] || mkdir -p ${_lcmtypes_c_dir}'
        COMMAND sh -c '${LCM_GEN_EXECUTABLE} ${lcmgen_c_flags}'
        DEPENDS ${_lcmtypes})

    # aggregate into a static library
    add_library(${libname} STATIC ${_lcmtypes_c_files})
    set_target_properties(${libname} PROPERTIES PREFIX "lib")
    set_target_properties(${libname} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    target_compile_options(${libname} PRIVATE -fPIC)
    target_include_directories(${libname} PUBLIC ${LCM_INCLUDE_DIRS} ${lcmtypes_include_build_path})

    # Copy the .h files into the output include/ path
    set(_output_h_files)
    foreach(_lcmtype_h_file ${_lcmtypes_h_files})
      file(RELATIVE_PATH _h_relname ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/c/lcmtypes ${_lcmtype_h_file})
      set(_h_file_output ${lcmtypes_include_build_path}/lcmtypes/${_h_relname})
      add_custom_command(OUTPUT ${_h_file_output}
                         COMMAND ${CMAKE_COMMAND} -E copy ${_lcmtype_h_file} ${_h_file_output}
                         DEPENDS ${_lcmtype_h_file})
      list(APPEND _output_h_files ${_h_file_output})
    endforeach()
    add_custom_target(${PROJECT_NAME}_lcmgen_output_h_files ALL DEPENDS ${_output_h_files})
    add_dependencies(${libname} ${PROJECT_NAME}_lcmgen_output_h_files)

    #    add_library("${libname}-static" STATIC ${_lcmtypes_c_files})
    #    set_source_files_properties(${_lcmtypes_c_files} PROPERTIES COMPILE_FLAGS "-I${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/c")
    #    set_target_properties("${libname}-static" PROPERTIES OUTPUT_NAME "${libname}")
    #    set_target_properties("${libname}-static" PROPERTIES PREFIX "lib")
    #    set_target_properties("${libname}-static" PROPERTIES CLEAN_DIRECT_OUTPUT 1)

    # make header files and libraries public
    #    pods_install_libraries(${libname})
    #    pods_install_headers(${_lcmtypes_h_files} DESTINATION lcmtypes)

    # set some compilation variables
    set(LCMTYPES_LIBS ${libname} PARENT_SCOPE)

    # create a pkg-config file
    #  	pods_install_pkg_config_file(${libname}
    #    	CFLAGS
    #    	DESCRIPTION "LCM types for ${PROJECT_NAME}"
    #        LIBS -l${libname}
    #    	REQUIRES lcm
    #    	VERSION 0.0.0)

    lcmtypes_add_clean_dir("${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/c")
endfunction()

function(lcmtypes_build_cpp)
    set(_lcmtypes ${ARGV})

    # set some defaults

    # generate CPP bindings for LCM types
    set(_lcmtypes_cpp_dir ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/cpp/lcmtypes)

    # blow away any existing auto-generated files.
    file(REMOVE_RECURSE ${_lcmtypes_cpp_dir})

    # run lcm-gen now
    execute_process(COMMAND mkdir -p ${_lcmtypes_cpp_dir})
    set(lcmgen_cpp_flags --lazy
                         --cpp
                         --cpp-hpath ${_lcmtypes_cpp_dir}
                         --cpp-include lcmtypes
                         ${_lcmtypes})
    lcmgen(${lcmgen_cpp_flags})

    # get a list of all generated .hpp files
    file(GLOB_RECURSE _lcmtypes_hpp_files  ${_lcmtypes_cpp_dir}/*.hpp)

    # run lcm-gen at compile time
    add_custom_command(OUTPUT ${_lcmtypes_hpp_files}
        COMMAND sh -c '[ -d ${_lcmtypes_cpp_dir} ] || mkdir -p ${_lcmtypes_cpp_dir}'
        COMMAND sh -c '${LCM_GEN_EXECUTABLE} ${lcmgen_cpp_flags}'
        DEPENDS ${_lcmtypes})
    add_custom_target(${PROJECT_NAME}_lcmgen_cpp ALL DEPENDS ${_lcmtypes_hpp_files})

    # Copy the .hpp files into the output include/ path
    set(_output_hpp_files)
    foreach(_lcmtype_hpp_file ${_lcmtypes_hpp_files})
      file(RELATIVE_PATH _hpp_relname ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/cpp/lcmtypes ${_lcmtype_hpp_file})
      set(_hpp_file_output ${lcmtypes_include_build_path}/lcmtypes/${_hpp_relname})
      add_custom_command(OUTPUT ${_hpp_file_output}
                         COMMAND ${CMAKE_COMMAND} -E copy ${_lcmtype_hpp_file} ${_hpp_file_output}
                         DEPENDS ${_lcmtype_hpp_file})
      list(APPEND _output_hpp_files ${_hpp_file_output})
    endforeach()
    add_custom_target(${PROJECT_NAME}_lcmgen_output_hpp_files ALL DEPENDS ${_output_hpp_files})

    lcmtypes_add_clean_dir("${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/cpp")
endfunction()

function(lcmtypes_build_java)
    set(_lcmtypes ${ARGV})

    # do we have Java?
    find_package(Java)
    if(JAVA_COMPILE STREQUAL JAVA_COMPILE-NOTFOUND OR
       JAVA_ARCHIVE STREQUAL JAVA_ARCHIVE-NOTFOUND)
        message(STATUS "Not building Java LCM type bindings (Can't find Java)")
        return()
    endif()

    if(NOT JAVA_OUTPUT_PATH)
      set(JAVA_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/java)
    endif()
    execute_process(COMMAND mkdir -p ${JAVA_OUTPUT_PATH})

    # do we have LCM java bindings?  where is lcm.jar?
    execute_process(COMMAND pkg-config --variable=classpath lcm-java OUTPUT_VARIABLE LCM_JAR_FILE)
    if(NOT LCM_JAR_FILE)
        message(STATUS "Not building Java LCM type bindings (Can't find lcm.jar)")
        return()
    endif()
    string(STRIP ${LCM_JAR_FILE} LCM_JAR_FILE)
    set(LCMTYPES_JAR ${JAVA_OUTPUT_PATH}/${PROJECT_NAME}_lcmtypes.jar)

    # generate Java bindings for LCM types
    set(_lcmtypes_java_dir ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/java)

    # blow away any existing auto-generated files?
    file(REMOVE_RECURSE ${_lcmtypes_java_dir})

    # run lcm-gen now
    execute_process(COMMAND mkdir -p ${_lcmtypes_java_dir})
    lcmgen(--lazy -j ${_lcmtypes} --jpath ${_lcmtypes_java_dir})

    # get a list of all generated .java files
    file(GLOB_RECURSE _lcmtypes_java_files ${_lcmtypes_java_dir}/*.java)

    # run lcm-gen at compile time
    add_custom_command(OUTPUT ${_lcmtypes_java_files}
        COMMAND sh -c '[ -d ${_lcmtypes_java_dir} ] || mkdir -p ${_lcmtypes_java_dir}'
        COMMAND sh -c '${LCM_GEN_EXECUTABLE} --lazy -j ${_lcmtypes} --jpath ${_lcmtypes_java_dir}'
        DEPENDS ${_lcmtypes})

    set(java_classpath ${_lcmtypes_java_dir}:${LCM_JAR_FILE})

    # search for lcmtypes_*.jar files in well-known places and add them to the
    # classpath
    foreach(pfx /usr /usr/local ${CMAKE_INSTALL_PREFIX})
        file(GLOB_RECURSE jarfiles ${pfx}/share/java/lcmtypes_*.jar)
        foreach(jarfile ${jarfiles})
            set(java_classpath ${java_classpath}:${jarfile})
            #            message("found ${jarfile}")
        endforeach()
    endforeach()

    # convert the list of .java filenames to a list of .class filenames
    foreach(javafile ${_lcmtypes_java_files})
        string(REPLACE .java .class __tmp_class_fname ${javafile})
        #        add_custom_command(OUTPUT ${__tmp_class_fname} COMMAND
        #            ${JAVA_COMPILE} -source 6 -cp ${_lcmtypes_java_dir}:${lcm_jar} ${javafile} VERBATIM DEPENDS ${javafile})
        list(APPEND _lcmtypes_class_files ${__tmp_class_fname})
        unset(__tmp_class_fname)
    endforeach()

    # add a rule to build the .class files from from the .java files
    add_custom_command(OUTPUT ${_lcmtypes_class_files} COMMAND
        ${JAVA_COMPILE} -source 6 -target 6 -cp ${java_classpath} ${_lcmtypes_java_files}
        DEPENDS ${_lcmtypes_java_files} VERBATIM)

    # add a rule to build a .jar file from the .class files
    add_custom_command(OUTPUT ${LCMTYPES_JAR} COMMAND
        ${JAVA_ARCHIVE} cf ${LCMTYPES_JAR} -C ${_lcmtypes_java_dir} . DEPENDS ${_lcmtypes_class_files} VERBATIM)
    add_custom_target(${PROJECT_NAME}_lcmtypes_jar ALL DEPENDS ${LCMTYPES_JAR})

    install(FILES ${LCMTYPES_JAR} DESTINATION share/java)
    set(LCMTYPES_JAR ${LCMTYPES_JAR} PARENT_SCOPE)

    lcmtypes_add_clean_dir(${_lcmtypes_java_dir})
endfunction()

function(lcmtypes_build_python)
    set(_lcmtypes ${ARGV})

    find_package(PythonInterp)
    if(NOT PYTHONINTERP_FOUND)
        message(STATUS "Not building Python LCM type bindings (Can't find Python)")
        return()
    endif()

    if(NOT PYTHON_OUTPUT_PATH)
      set(PYTHON_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/python)
    endif()

    # Get python version
    execute_process(COMMAND
        ${PYTHON_EXECUTABLE} -c "import sys; sys.stdout.write(sys.version[:3])"
        OUTPUT_VARIABLE pyversion)

    set(_lcmtypes_python_dir ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/python)

    # purge existing files?
    file(REMOVE_RECURSE ${_lcmtypes_python_dir})

    # generate Python bindings for LCM types
    execute_process(COMMAND mkdir -p ${_lcmtypes_python_dir})
    execute_process(COMMAND ${LCM_GEN_EXECUTABLE} --lazy -p ${_lcmtypes} --ppath ${_lcmtypes_python_dir})

    # get a list of all generated .py files
    file(GLOB_RECURSE _lcmtypes_py_files ${_lcmtypes_python_dir}/*.py)

    # run lcm-gen at compile time
    add_custom_command(OUTPUT ${_lcmtypes_py_files}
      COMMAND sh -c '${LCM_GEN_EXECUTABLE} --lazy -p ${_lcmtypes} --ppath ${_lcmtypes_python_dir}'
      DEPENDS ${_lcmtypes})
    add_custom_target(${PROJECT_NAME}_lcmgen_python ALL DEPENDS ${_lcmtypes_py_files})

    # Copy the .py files into the output python path, and generate install rules.
    set(_output_py_files)
    foreach(_lcmtype_py_file ${_lcmtypes_py_files})
      file(RELATIVE_PATH _py_relname ${CMAKE_CURRENT_BINARY_DIR}/lcmtypes/python ${_lcmtype_py_file})
      get_filename_component(_py_reldir ${_py_relname} DIRECTORY)
      set(_py_file_output ${PYTHON_OUTPUT_PATH}/${_py_relname})
      add_custom_command(OUTPUT ${_py_file_output}
                         COMMAND ${CMAKE_COMMAND} -E copy ${_lcmtype_py_file} ${_py_file_output}
                         DEPENDS ${_lcmtype_py_file})
      list(APPEND _output_py_files ${_py_file_output})

      install(FILES ${_lcmtype_py_file}
             DESTINATION lib/python${pyversion}/dist-packages/${_py_reldir})
    endforeach()
    add_custom_target(${PROJECT_NAME}_lcmgen_output_py_files ALL DEPENDS ${_output_py_files})

    lcmtypes_add_clean_dir(${_lcmtypes_python_dir})
endfunction()

function(lcmtypes_build)
    # Get the list of .lcm files to build
    set(lcmtypes)
    set(lcmtypes_include_build_path)
    set(modewords INCLUDE_BUILD_PATH TYPES)
    set(curmode "")
    foreach(word ${ARGV})
        list(FIND modewords ${word} mode_index)
        if(${mode_index} GREATER -1)
            set(curmode ${word})
        elseif(curmode STREQUAL INCLUDE_BUILD_PATH)
            set(lcmtypes_include_build_path ${word})
        elseif(curmode STREQUAL TYPES)
            if(EXISTS ${word})
                list(APPEND lcmtypes ${word})
            elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${word})
                list(APPEND lcmtypes ${CMAKE_CURRENT_SOURCE_DIR}/${word})
            else()
                message(FATAL_ERROR "Unable to find ${word}")
            endif()
        endif()
    endforeach()

    list(LENGTH lcmtypes _num_lcmtypes)
    if(_num_lcmtypes EQUAL 0)
        message("No LCM types specified")
        return()
    endif()

    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LCM REQUIRED lcm)

    # find lcm-gen (it may be in the install path)
    find_program(LCM_GEN_EXECUTABLE lcm-gen ${EXECUTABLE_OUTPUT_PATH} ${EXECUTABLE_INSTALL_PATH})
    if (NOT LCM_GEN_EXECUTABLE)
    	message(FATAL_ERROR "lcm-gen not found!\n")
    	return()
    endif()

    if(NOT lcmtypes_include_build_path)
      set(lcmtypes_include_build_path ${CMAKE_CURRENT_BINARY_DIR}/include)
      execute_process(COMMAND mkdir -p ${lcmtypes_include_build_path})
    endif()

    lcmtypes_build_c(${lcmtypes})
    lcmtypes_build_cpp(${lcmtypes})

    lcmtypes_build_java(${lcmtypes})
    lcmtypes_build_python(${lcmtypes})
    install(FILES ${lcmtypes} DESTINATION share/lcmtypes)
endfunction()
