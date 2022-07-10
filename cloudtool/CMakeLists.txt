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
    device/photoneo.cpp
)

set(Hdrs
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
    device/photoneo.h
)

set(QRCs resources/res.qrc)

set(IRCs resources/logo/logo.rc)

find_package(VTK COMPONENTS
    vtkGUISupportQt
    vtkGUISupportQtOpenGL
    vtkRenderingGL2PSOpenGL2
    vtkRenderingVolumeOpenGL2)

add_executable(cloudtool WIN32 ${Srcs} ${Hdrs} ${QRCs} ${IRCs})

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(DEBUG_SUFFIX "d")
endif()

target_link_libraries(cloudtool ct_base${DEBUG_SUFFIX} ct_modules${DEBUG_SUFFIX})
target_link_libraries(cloudtool PhoXi_API_msvc14_${CMAKE_BUILD_TYPE}_1.4.1 k4a k4arecord)
target_link_libraries(cloudtool Qt${QT_VERSION_MAJOR}::Widgets ${VTK_LIBRARIES} ${PCL_LIBRARIES})

if(WIN32)
    # 1. QT DLL
    set(QT_INSTALL_PATH "${CMAKE_PREFIX_PATH}")

    if(NOT EXISTS "${QT_INSTALL_PATH}/bin")
        set(QT_INSTALL_PATH "${QT_INSTALL_PATH}/..")

        if(NOT EXISTS "${QT_INSTALL_PATH}/bin")
            set(QT_INSTALL_PATH "${QT_INSTALL_PATH}/..")
        endif()
    endif()

    add_custom_command(TARGET cloudtool POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:cloudtool>/platforms/"
        COMMAND ${CMAKE_COMMAND} -E copy "${QT_INSTALL_PATH}/plugins/platforms/qwindows${DEBUG_SUFFIX}.dll"
        "$<TARGET_FILE_DIR:cloudtool>/platforms/"
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:cloudtool>/iconengines/"
        COMMAND ${CMAKE_COMMAND} -E copy "${QT_INSTALL_PATH}/plugins/iconengines/qsvgicon${DEBUG_SUFFIX}.dll"
        "$<TARGET_FILE_DIR:cloudtool>/iconengines/"
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:cloudtool>/styles/"
        COMMAND ${CMAKE_COMMAND} -E copy "${QT_INSTALL_PATH}/plugins/styles/qwindowsvistastyle${DEBUG_SUFFIX}.dll"
        "$<TARGET_FILE_DIR:cloudtool>/styles/")

    foreach(QT_LIB Core Gui Widgets Svg)
        add_custom_command(TARGET cloudtool POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "${QT_INSTALL_PATH}/bin/Qt5${QT_LIB}${DEBUG_SUFFIX}.dll"
            "$<TARGET_FILE_DIR:cloudtool>")
    endforeach(QT_LIB)

    # 2. Camera DLL
    foreach(KINECT_LIB k4a k4arecord depthengine_2_0)
        add_custom_command(TARGET cloudtool POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "${AzureKinect_DIR}/bin/${KINECT_LIB}.dll" "$<TARGET_FILE_DIR:cloudtool>"
            COMMAND ${CMAKE_COMMAND} -E copy "${AzureKinect_DIR}/netstandard2.0/Microsoft.Azure.Kinect.Sensor.dll" "$<TARGET_FILE_DIR:cloudtool>")
    endforeach(KINECT_LIB)

    add_custom_command(TARGET cloudtool POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${Photoneo_DIR}/bin/PhoXi_API_msvc14_${CMAKE_BUILD_TYPE}_1.4.1.dll" "$<TARGET_FILE_DIR:cloudtool>")
endif()

install(TARGETS cloudtool)
install(DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE}/ 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        FILES_MATCHING PATTERN "*.dll")