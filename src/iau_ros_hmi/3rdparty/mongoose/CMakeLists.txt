option(WSOCKET_BUILD_UTILS "Build utils for woskcet" ON)

add_library(mongoose mongoose.c mongoose.h)

if (WIN32)
target_link_libraries(mongoose
    ws2_32
)
endif()

