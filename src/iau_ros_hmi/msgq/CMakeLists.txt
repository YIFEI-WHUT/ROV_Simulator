add_library(msgq msgq.c msgq.h)
target_link_libraries(msgq
    ${NNG_LIBS}
)
