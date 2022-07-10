set(LibSrcs
        features.cpp
        filters.cpp
        keypoints.cpp
        recognition.cpp
        registration.cpp
        segmentation.cpp
        surface.cpp
        )

set(LibHdrs
        features.h
        filters.h
        keypoints.h
        recognition.h
        registration.h
        segmentation.h
        surface.h
        )

add_library(ct_modules SHARED ${LibSrcs} ${LibHdrs})

target_link_libraries(ct_modules PRIVATE ct_base Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

target_compile_definitions(ct_modules PRIVATE CT_LIBRARY)

set_target_properties(ct_modules PROPERTIES PUBLIC_HEADER "${LibHdrs}")

install(TARGETS ct_modules PUBLIC_HEADER DESTINATION include/modules)