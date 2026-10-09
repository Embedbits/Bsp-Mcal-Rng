#======================== Unit tests configuration ============================#
# Unit tests of Rng module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(RNG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${RNG_DIR}/.." ABSOLUTE)

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
