add_library(srrg_boss SHARED)
add_library(wombat_srrg::srrg2_core::srrg_boss ALIAS srrg_boss)

target_sources(
  srrg_boss
  PRIVATE
    src/blob.cpp
    src/deserializer.cpp
    src/id_context.cpp 
    src/identifiable.cpp
    src/id_placeholder.cpp
    src/json_recursive_object_parser.cpp
    src/json_object_writer.cpp
    src/object_data.cpp
    src/object_parser.cpp
    src/object_writer.cpp
    src/serializable.cpp
    src/serialization_context.cpp
    src/serializer.cpp
    src/stream.cpp
)

target_include_directories(
  srrg_boss
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_boss
  PUBLIC
    Eigen3::Eigen
)

wombat_install_target(srrg_boss)
