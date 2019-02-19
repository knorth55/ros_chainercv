cmake_minimum_required(VERSION 2.8.12)
project(ros_chainercv)

# Make sure to find-package `catkin_virtualenv`
find_package(catkin REQUIRED catkin_virtualenv)

catkin_python_setup()

# Must be called before catkin_generate_virtualenv
catkin_package()

# Generate the virtualenv, optionally with python 3 as the default interpreter:
catkin_generate_virtualenv()
# catkin_generate_virtualenv(PYTHON3)

install(DIRECTORY
  launch node_scripts sample test yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_python()
  find_package(jsk_tools REQUIRED)
  # too slow
  # add_rostest(test/test_faster_rcnn_fpn_resnet101_object_detection.test)
  # add_rostest(test/test_faster_rcnn_fpn_resnet50_object_detection.test)
  add_rostest(test/test_faster_rcnn_vgg16_object_detection.test)
  add_rostest(test/test_ssd300_object_detection.test)
  add_rostest(test/test_ssd512_object_detection.test)
  add_rostest(test/test_yolo_v2_object_detection.test)
  add_rostest(test/test_yolo_v2_tiny_object_detection.test)
  add_rostest(test/test_yolo_v3_object_detection.test)

endif()

