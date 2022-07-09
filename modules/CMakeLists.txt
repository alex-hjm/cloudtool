set(LibSrcs
        features.cpp
        filters.cpp
        keypoints.cpp
        recognition.cpp
        registration.cpp
        segmentation.cpp
        surface.cpp
        )

set(MocHeaders
        features.h
        filters.h
        keypoints.h
        recognition.h
        registration.h
        segmentation.h
        surface.h
        )

qt5_wrap_cpp(LibMocSrcs ${MocHeaders})

add_library(ct_modules SHARED ${LibSrcs} ${LibMocSrcs} ${MocHeaders})

set_target_properties(ct_modules PROPERTIES PUBLIC_HEADER "${MocHeaders}")

target_link_libraries(ct_modules PRIVATE ct_base Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

target_compile_definitions(ct_modules PRIVATE CT_LIBRARY)

install(TARGETS ct_modules
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include/modules)