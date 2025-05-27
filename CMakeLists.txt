cmake_minimum_required(VERSION 3.15)
project(DeliverySystem)

# Find GTK3 using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

# Include directories
include_directories(
    ${GTK3_INCLUDE_DIRS}
)

# Add executable
add_executable(delivery_system delivery_system.c)

# Link libraries
target_link_libraries(delivery_system
    ${GTK3_LIBRARIES}
    m  # Link math library (-lm)
)

# Set compiler flags
target_compile_options(delivery_system PRIVATE ${GTK3_CFLAGS_OTHER})