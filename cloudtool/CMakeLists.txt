file(GLOB Srcs *.cpp */*.cpp)

file(GLOB Hdrs *.h */*.h)

file(GLOB Uis *.ui */*.ui)

set(QRCs resources/res.qrc)

set(IRCs resources/logo/logo.rc)

set(TRCs resources/trans/zh_CN.ts)

set_source_files_properties(${TRCs} PROPERTIES OUTPUT_LOCATION resources/trans/)

if(${QT_VERSION_MAJOR} STREQUAL "5")
    qt5_create_translation(Trs ${Uis} ${TRCs})
else()
    qt6_create_translation(Trs ${Uis} ${TRCs})
endif()

add_executable(cloudtool WIN32 ${Srcs} ${Hdrs} ${QRCs} ${IRCs} ${Trs})

target_link_libraries(cloudtool  PRIVATE ct_base Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

add_custom_command(TARGET cloudtool POST_BUILD
    COMMAND ${QT_DIR}/../../../bin/windeployqt --no-compiler-runtime $<TARGET_FILE:cloudtool>
)