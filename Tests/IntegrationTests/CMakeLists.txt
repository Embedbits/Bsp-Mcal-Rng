#===================== Integration tests configuration ========================#
# Integration tests of Rng module on target (any STM32F7 board). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_Rng.c and
# all libraries of the firmware are listed here.

# MCUs without RNG peripheral (CMSIS device header defines no RNG - every STM32F7
# has RNG) have empty Rng_Lib - the test set is not created.
get_target_property(RNG_IT_RAL_DEFINES  Ral_Lib INTERFACE_COMPILE_DEFINITIONS)
get_target_property(RNG_IT_RAL_INCLUDES Ral_Lib INTERFACE_INCLUDE_DIRECTORIES)
list(FILTER RNG_IT_RAL_DEFINES INCLUDE REGEX "^STM32")
string(TOLOWER "${RNG_IT_RAL_DEFINES}.h" RNG_IT_DEVICE_HEADER)
find_file(RNG_IT_DEVICE_HEADER_PATH "${RNG_IT_DEVICE_HEADER}"
    PATHS ${RNG_IT_RAL_INCLUDES} NO_DEFAULT_PATH NO_CACHE REQUIRED)
file(STRINGS "${RNG_IT_DEVICE_HEADER_PATH}" RNG_IT_RNG_DEFINE REGEX "^#define[ \t]+RNG[ \t]")

if(NOT RNG_IT_RNG_DEFINE)
    message(STATUS "Rng integration tests skipped - ${RNG_IT_RAL_DEFINES} has no RNG peripheral")
    return()
endif()

# StartUp calls BspMain() - the entry point of the test firmware is provided by
# the test set (BspMain.h, ItTarget_Rng.c)
if(NOT TARGET BspMain_Lib)
    add_library(BspMain_Lib INTERFACE)
    target_include_directories(BspMain_Lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

IntegrationTesting_Add_Test(
    NAME            Rng
    TEST_SOURCE     ItTest_Rng.c
    SOURCES         ItTarget_Rng.c      # Target interface of the test set
    LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                    Nvic_Lib            # Fault handlers, system reset (ItTarget)
                    Rng_Lib
    TIMEOUT         60
)
