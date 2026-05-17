cmake_minimum_required(VERSION 3.19)
project(Sim_Autito LANGUAGES C CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets)

qt_standard_project_setup()

qt_add_executable(Sim_Autito
    WIN32 MACOSX_BUNDLE
    app/main.cpp
    app/mainwindow.cpp
    app/mainwindow.h
    sim/sim_world.cpp
    sim/sim_world.h
    sim/sim_robot.cpp
    sim/sim_robot.h
    nav/nav_core.c
    nav/nav_core.h
    nav/nav_map.c
    nav/nav_map.h
    nav/nav_flood.c
    nav/nav_flood.h
    nav/nav_frontier_eval.c
    nav/nav_frontier_eval.h
    nav/nav_goal_return_eval.c
    nav/nav_goal_return_eval.h
    nav/nav_supervisor.c
    nav/nav_supervisor.h
    nav/pid_controller.c
    nav/pid_controller.h
    nav/nav_types.h
)

target_include_directories(Sim_Autito
    PRIVATE
        app
        sim
        nav
)

target_compile_definitions(Sim_Autito
    PRIVATE
        SIM_AUTITO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
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
