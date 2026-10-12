cmake_minimum_required(VERSION 3.16)
project(biped_s52)

include(GNUInstallDirs)

foreach(dir config launch meshes urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/${dir})
endforeach()
