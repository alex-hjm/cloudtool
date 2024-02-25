file(GLOB LibSrcs *.cpp)

file(GLOB LibHdrs *.h)

add_executable(test_cloudview WIN32 ${LibSrcs} ${LibHdrs})

target_link_libraries(test_cloudview  PRIVATE ct_base Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

add_custom_command(TARGET test_cloudview POST_BUILD
    COMMAND ${QT_DIR}/../../../bin/windeployqt --no-compiler-runtime $<TARGET_FILE:test_cloudview>
)