project(tests)

add_executable(data_serialize_test ${tests_SOURCE_DIR}/data_serialize_test.cpp)
target_link_libraries(data_serialize_test ${LIBOPENCV_LDFLAGS})

add_executable(data_serialize_test2 ${tests_SOURCE_DIR}/data_serialize_test2.cpp)
target_link_libraries(data_serialize_test2 ${LIBOPENCV_LDFLAGS} ${Boost_LIBRARIES})

add_executable(data_serialize_test3 ${tests_SOURCE_DIR}/data_serialize_test3.cpp)
target_link_libraries(data_serialize_test3 ${LIBOPENCV_LDFLAGS} ${Boost_LIBRARIES})

add_executable(data_serialize_test4 ${tests_SOURCE_DIR}/data_serialize_test4.cpp)
target_link_libraries(data_serialize_test4 ${Boost_LIBRARIES})

add_executable(data_deflate_test ${tests_SOURCE_DIR}/data_deflate_test.cpp)
target_link_libraries(data_deflate_test ${Boost_LIBRARIES})

add_executable(homography_test ${tests_SOURCE_DIR}/homography_test.cpp)
target_link_libraries(homography_test ${LIBOPENCV_LDFLAGS})

add_executable(homography_test2 ${tests_SOURCE_DIR}/homography_test2.cpp)
target_link_libraries(homography_test2 ${LIBOPENCV_LDFLAGS})

add_executable(resize_test ${tests_SOURCE_DIR}/resize_test.cpp)
target_link_libraries(resize_test ${LIBOPENCV_LDFLAGS})

add_executable(images_matcher_test ${tests_SOURCE_DIR}/images_matcher_test.cpp)
target_link_libraries(images_matcher_test ${LIBOPENCV_LDFLAGS} ${Boost_LIBRARIES})
