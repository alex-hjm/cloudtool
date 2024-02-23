file(GLOB Srcs *.cpp */*.cpp)

file(GLOB Hdrs *.h */*.h)

file(GLOB Uis *.ui */*.ui)

set(QRCs resources/res.qrc)

set(IRCs resources/logo/logo.rc)

set(TRCs resources/trans/zh_CN.ts)

set_source_files_properties(${TRCs} PROPERTIES OUTPUT_LOCATION resources/trans/)

qt5_create_translation(Trs ${Uis} ${TRCs})

add_executable(cloudtool WIN32 ${Srcs} ${Hdrs} ${QRCs} ${IRCs} ${Trs})

target_link_libraries(cloudtool  PRIVATE ct_base Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

add_custom_command(TARGET cloudtool POST_BUILD
    COMMAND ${Qt5_DIR}/../../../bin/windeployqt --no-compiler-runtime $<TARGET_FILE:cloudtool>
)