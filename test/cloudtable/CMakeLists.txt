file(GLOB LibSrcs *.cpp)

file(GLOB LibHdrs *.h)

add_executable(test_cloudtable WIN32 ${LibSrcs} ${LibHdrs})

target_link_libraries(test_cloudtable  PRIVATE ct_base Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

add_custom_command(TARGET test_cloudtable POST_BUILD
    COMMAND ${Qt5_DIR}/../../../bin/windeployqt --no-compiler-runtime $<TARGET_FILE:test_cloudtable>
)