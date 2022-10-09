cmake_minimum_required(VERSION 3.5)
project(wombat_srrg)

find_package(wombat_cmake REQUIRED)

wombat_package()

add_subdirectory(srrg_hbst)

wombat_package_install()
