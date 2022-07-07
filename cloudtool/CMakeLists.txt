set(Srcs 
        main.cpp
        mainwindow.cpp
        help/about.cpp
        help/shortcutkey.cpp
        edit/color.cpp
        edit/boundingbox.cpp
        edit/coordinate.cpp
        edit/normals.cpp
        edit/scale.cpp
        edit/transformation.cpp
        tool/filters.cpp
        tool/keypoints.cpp
        tool/rangeimage.cpp
        tool/segmentation.cpp
        tool/cutting.cpp
        tool/boundary.cpp
        tool/sampling.cpp
        tool/descriptor.cpp
        tool/surface.cpp
        tool/registration.cpp
        tool/pickpoints.cpp
        )

set(MOC_Hdrs 
        mainwindow.h
        help/about.h
        help/shortcutkey.h
        edit/color.h 
        edit/boundingbox.h
        edit/coordinate.h
        edit/normals.h
        edit/scale.h
        edit/transformation.h
        tool/filters.h
        tool/keypoints.h
        tool/rangeimage.h
        tool/segmentation.h
        tool/cutting.h
        tool/boundary.h
        tool/sampling.h
        tool/descriptor.h
        tool/surface.h
        tool/registration.h
        tool/pickpoints.h
        )

set(UIs 
        mainwindow.ui 
        )
        
set(QRCs resources/res.qrc)     

set(IRCs resources/logo/logo.rc)

find_package(VTK COMPONENTS 
        vtkGUISupportQt
        vtkGUISupportQtOpenGL
        vtkRenderingGL2PSOpenGL2
        vtkRenderingVolumeOpenGL2)

qt5_add_resources(QRC_Srcs ${QRCs})

add_executable(cloudtool WIN32 ${Srcs} ${MOC_Hdrs} ${UIs} ${QRC_Srcs} ${IRCs})

target_link_libraries(cloudtool PRIVATE ct_base ct_modules Qt${QT_VERSION_MAJOR}::Widgets ${VTK_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS cloudtool
        CONFIGURATIONS Debug
        RUNTIME DESTINATION Debug/bin)

install(TARGETS cloudtool
        CONFIGURATIONS Release
        RUNTIME DESTINATION Release/bin)
        
if (WIN32)
    set(DEBUG_SUFFIX)
    if (CMAKE_BUILD_TYPE MATCHES "Debug")
        set(DEBUG_SUFFIX "d")
    endif ()
    set(QT_INSTALL_PATH "${CMAKE_PREFIX_PATH}")
    if (NOT EXISTS "${QT_INSTALL_PATH}/bin")
        set(QT_INSTALL_PATH "${QT_INSTALL_PATH}/..")
        if (NOT EXISTS "${QT_INSTALL_PATH}/bin")
            set(QT_INSTALL_PATH "${QT_INSTALL_PATH}/..")
        endif ()
    endif ()
    if (EXISTS "${QT_INSTALL_PATH}/plugins/platforms/qwindows${DEBUG_SUFFIX}.dll")
        add_custom_command(TARGET cloudtool POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:cloudtool>/platforms/"
                COMMAND ${CMAKE_COMMAND} -E copy "${QT_INSTALL_PATH}/plugins/platforms/qwindows${DEBUG_SUFFIX}.dll"
                "$<TARGET_FILE_DIR:cloudtool>/platforms/"
                COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:cloudtool>/iconengines/"
                COMMAND ${CMAKE_COMMAND} -E copy "${QT_INSTALL_PATH}/plugins/iconengines/qsvgicon${DEBUG_SUFFIX}.dll"
                "$<TARGET_FILE_DIR:cloudtool>/iconengines/"
                COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:cloudtool>/styles/"
                COMMAND ${CMAKE_COMMAND} -E copy "${QT_INSTALL_PATH}/plugins/styles/qwindowsvistastyle${DEBUG_SUFFIX}.dll"
                "$<TARGET_FILE_DIR:cloudtool>/styles/"
                )
    endif ()
    foreach (QT_LIB Core Gui Widgets Svg)
        add_custom_command(TARGET cloudtool POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                "${QT_INSTALL_PATH}/bin/Qt5${QT_LIB}${DEBUG_SUFFIX}.dll"
                "$<TARGET_FILE_DIR:cloudtool>")
    endforeach (QT_LIB)
endif ()
