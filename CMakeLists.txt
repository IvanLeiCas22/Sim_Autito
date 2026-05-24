cmake_minimum_required(VERSION 3.19)
project(Sim_Autito LANGUAGES C CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets)

qt_standard_project_setup()

set(SIM_AUTITO_SOURCES
    app/main.cpp
    app/sim_mainwindow.cpp
    app/sim_mainwindow.h
    app/firmware_sim_bridge.cpp
    app/firmware_sim_bridge.h
    sim/sim_world.cpp
    sim/sim_world.h
    sim/sim_robot.cpp
    sim/sim_robot.h
)

set(SIM_AUTITO_FIRMWARE_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/firmware_core")
set(SIM_AUTITO_FIRMWARE_CORE_APP_NAV "${SIM_AUTITO_FIRMWARE_CORE_DIR}/app_nav.c")

if(EXISTS "${SIM_AUTITO_FIRMWARE_CORE_APP_NAV}")
    set(SIM_AUTITO_HAS_FIRMWARE_CORE 1)
    list(APPEND SIM_AUTITO_SOURCES
        firmware_core/app_nav.c
    )

    set(SIM_AUTITO_FIRMWARE_CORE_OPTIONAL_SOURCES
        firmware_core/pid_controller.c
        firmware_core/app_maze.c
    )

    foreach(optional_firmware_source IN LISTS SIM_AUTITO_FIRMWARE_CORE_OPTIONAL_SOURCES)
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${optional_firmware_source}")
            list(APPEND SIM_AUTITO_SOURCES
                ${optional_firmware_source}
            )
        endif()
    endforeach()
else()
    set(SIM_AUTITO_HAS_FIRMWARE_CORE 0)
endif()

qt_add_executable(Sim_Autito
    WIN32 MACOSX_BUNDLE
    ${SIM_AUTITO_SOURCES}
)

target_include_directories(Sim_Autito
    PRIVATE
        app
        sim
)

if(SIM_AUTITO_HAS_FIRMWARE_CORE)
    target_include_directories(Sim_Autito
        PRIVATE
            firmware_core
    )
endif()

target_compile_definitions(Sim_Autito
    PRIVATE
        SIM_AUTITO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
        SIM_AUTITO_HAS_FIRMWARE_CORE=${SIM_AUTITO_HAS_FIRMWARE_CORE}
)

target_link_libraries(Sim_Autito
    PRIVATE
        Qt::Core
        Qt::Widgets
)

add_custom_command(TARGET Sim_Autito POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/data"
        "$<TARGET_FILE_DIR:Sim_Autito>/data"
)

include(GNUInstallDirs)

install(TARGETS Sim_Autito
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET Sim_Autito
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
