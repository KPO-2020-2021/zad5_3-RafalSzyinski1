set(TEST_BINARY ${CMAKE_PROJECT_NAME}_tst)
file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.h *.cpp)

add_executable(${TEST_BINARY} ${TEST_SOURCES})

enable_testing()
add_test(NAME ${TEST_BINARY} COMMAND ${TEST_BINARY})
target_link_libraries(${TEST_BINARY} PRIVATE ${CMAKE_PROJECT_NAME}_lib gtest)