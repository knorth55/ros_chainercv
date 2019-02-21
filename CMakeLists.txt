cmake_minimum_required(VERSION 2.8.12)
project(ros_chainercv)

# Make sure to find-package `catkin_virtualenv`
find_package(catkin REQUIRED catkin_virtualenv)

catkin_python_setup()

# Must be called before catkin_generate_virtualenv
catkin_package()

# Generate the virtualenv, optionally with python 3 as the default interpreter:
catkin_generate_virtualenv(
  PYTHON_VERSION_MAJOR 2
  USE_SYSTEM_PACKAGES TRUE
  ISOLATE_REQUIREMENTS FALSE
)
# catkin_generate_virtualenv(PYTHON3)

file(GLOB NODE_SCRIPTS_FILES node_scripts/*)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/install_pretrained_models.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY launch sample test yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_custom_target(${PROJECT_NAME}_install_pretrained_models ALL
  COMMAND ${${PROJECT_NAME}_VENV_DEVEL_DIR}/bin/python ${PROJECT_SOURCE_DIR}/scripts/install_pretrained_models.py
  DEPENDS ${PROJECT_NAME}_generate_virtualenv
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(jsk_tools REQUIRED)
  roslint_python()
  macro(virtualenv_add_rostest rostest_file)
    add_rostest(${rostest_file}
      DEPENDENCIES
      ${PROJECT_NAME}_generate_virtualenv
      ${PROJECT_NAME}_install_pretrained_models
    )
    roslaunch_add_file_check(${rostest_file})
  endmacro()

  # comment out too slow
  # virtualenv_add_rostest(test/test_faster_rcnn_fpn_resnet101_object_detection.test)
  # virtualenv_add_rostest(test/test_faster_rcnn_fpn_resnet50_object_detection.test)
  virtualenv_add_rostest(test/test_faster_rcnn_vgg16_object_detection.test)
  # virtualenv_add_rostest(test/test_fcis_resnet101_instance_segmentation.test)
  virtualenv_add_rostest(test/test_pspnet_resnet101_semantic_segmentation.test)
  virtualenv_add_rostest(test/test_pspnet_resnet50_semantic_segmentation.test)
  virtualenv_add_rostest(test/test_ssd300_object_detection.test)
  virtualenv_add_rostest(test/test_ssd512_object_detection.test)
  virtualenv_add_rostest(test/test_yolo_v2_object_detection.test)
  virtualenv_add_rostest(test/test_yolo_v2_tiny_object_detection.test)
  virtualenv_add_rostest(test/test_yolo_v3_object_detection.test)

endif()

