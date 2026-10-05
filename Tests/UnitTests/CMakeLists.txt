#======================== Unit tests configuration ============================#
# Unit tests of Rng module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(RNG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${RNG_DIR}/.." ABSOLUTE)

# MCUs without RNG peripheral (e.g. STM32F401, STM32F411, STM32F446 - CMSIS device
# header defines no RNG) have empty Rng_Lib - the test set is not created.
get_target_property(RNG_UT_RAL_DEFINES  Ral_Lib INTERFACE_COMPILE_DEFINITIONS)
get_target_property(RNG_UT_RAL_INCLUDES Ral_Lib INTERFACE_INCLUDE_DIRECTORIES)
list(FILTER RNG_UT_RAL_DEFINES INCLUDE REGEX "^STM32")
string(TOLOWER "${RNG_UT_RAL_DEFINES}.h" RNG_UT_DEVICE_HEADER)
find_file(RNG_UT_DEVICE_HEADER_PATH "${RNG_UT_DEVICE_HEADER}"
    PATHS ${RNG_UT_RAL_INCLUDES} NO_DEFAULT_PATH NO_CACHE REQUIRED)
file(STRINGS "${RNG_UT_DEVICE_HEADER_PATH}" RNG_UT_RNG_DEFINE REGEX "^#define[ \t]+RNG[ \t]")

if(NOT RNG_UT_RNG_DEFINE)
    message(STATUS "Rng unit tests skipped - ${RNG_UT_RAL_DEFINES} has no RNG peripheral")
    return()
endif()

UnitTesting_Add_Test(
    NAME            Rng
    TEST_SOURCE     Test_Rng.c
    UUT_SOURCES     ${RNG_DIR}/Rng.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
    INCLUDE_DIRS    ${RNG_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
)
