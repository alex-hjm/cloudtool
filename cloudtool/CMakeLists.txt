file(GLOB Srcs *.cpp */*.cpp)
file(GLOB Hdrs *.h */*.h)
file(GLOB Uis *.ui */*.ui)

set(QRC ${CMAKE_CURRENT_SOURCE_DIR}/resources/res.qrc)
set(RC  ${CMAKE_CURRENT_SOURCE_DIR}/resources/logo/logo.rc)
set(TS  ${CMAKE_CURRENT_SOURCE_DIR}/resources/trans/zh_CN.ts)

# if(${QT_VERSION_MAJOR} STREQUAL "5")
#     qt5_create_translation(Trs ${Uis} ${Srcs} ${TRAs})
#     add_executable(cloudtool ${Srcs} ${Hdrs} ${QRCs} ${IRCs} ${Trs})
# else()
#     qt_add_executable(cloudtool ${Srcs} ${Hdrs} ${QRCs} ${IRCs})
#     qt_add_translations(cloudtool TS_FILES cloudtool_zh.ts)
# endif()

add_executable(cloudtool ${Srcs} ${Hdrs} ${QRC} ${RC})

target_link_libraries(cloudtool PRIVATE ct_base ct_module Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

add_custom_command(TARGET cloudtool PRE_BUILD 
    COMMAND ${QT_DIR}/../../../bin/lupdate ${Srcs} ${Uis} -ts ${TS} -no-obsolete -locations none
)

add_custom_command(TARGET cloudtool PRE_BUILD 
    COMMAND ${QT_DIR}/../../../bin/lrelease ${TS}
)

add_custom_command(TARGET cloudtool POST_BUILD
    COMMAND ${QT_DIR}/../../../bin/windeployqt --no-compiler-runtime --no-translations $<TARGET_FILE:cloudtool>
)