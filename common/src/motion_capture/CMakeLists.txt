
add_library(motion_capture SHARED
  natnet_messages.cpp
  data_model.cpp
  optitrack.cpp
  udphdl.cpp
)

install(TARGETS motion_capture
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)
