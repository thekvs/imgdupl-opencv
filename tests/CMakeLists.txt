project(tests)

add_executable(data_serialize_test ${tests_SOURCE_DIR}/data_serialize_test.cpp)
add_dependencies(data_serialize_test opencv)
target_link_libraries(data_serialize_test ${OpenCV_LIBS})

add_executable(data_serialize_test2 ${tests_SOURCE_DIR}/data_serialize_test2.cpp)
add_dependencies(data_serialize_test2 opencv)
target_link_libraries(data_serialize_test2 ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(data_serialize_test3 ${tests_SOURCE_DIR}/data_serialize_test3.cpp)
add_dependencies(data_serialize_test3 opencv)
target_link_libraries(data_serialize_test3 ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(data_serialize_test4 ${tests_SOURCE_DIR}/data_serialize_test4.cpp)
add_dependencies(data_serialize_test4 opencv)
target_link_libraries(data_serialize_test4 ${Boost_LIBRARIES})

add_executable(data_deflate_test ${tests_SOURCE_DIR}/data_deflate_test.cpp)
target_link_libraries(data_deflate_test ${Boost_LIBRARIES})

add_executable(homography_test ${tests_SOURCE_DIR}/homography_test.cpp)
add_dependencies(homography_test opencv)
target_link_libraries(homography_test ${OpenCV_LIBS})

add_executable(homography_test2 ${tests_SOURCE_DIR}/homography_test2.cpp)
add_dependencies(homography_test2 opencv)
target_link_libraries(homography_test2 ${OpenCV_LIBS})

add_executable(resize_test ${tests_SOURCE_DIR}/resize_test.cpp)
add_dependencies(resize_test opencv)
target_link_libraries(resize_test ${OpenCV_LIBS})

add_executable(images_matcher_test ${tests_SOURCE_DIR}/images_matcher_test.cpp)
add_dependencies(images_matcher_test opencv)
target_link_libraries(images_matcher_test ${OpenCV_LIBS} ${Boost_LIBRARIES})
