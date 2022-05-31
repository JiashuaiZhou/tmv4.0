add_library(po STATIC
    "${PROJECT_SOURCE_DIR}/dependencies/program-options-lite/program_options_lite.cpp"
)

target_include_directories(po INTERFACE
    "${PROJECT_SOURCE_DIR}/dependencies/"
)

add_all_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR} )
