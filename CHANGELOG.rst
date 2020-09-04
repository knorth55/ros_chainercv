^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_chainercv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2020-09-05)
------------------
* update Dockerfile
* update .travis to 0.5.13
* use catkin_virtualenv>=0.6.1
* update catkin_virtualenv
* update chainer version
* Merge pull request `#2 <https://github.com/knorth55/ros_chainercv/issues/2>`_ from knorth55/update-travis
  update travis
* update travis
* use typing <= 3.6.6 for python2
  https://github.com/chainer/chainer/pull/7564
* update dockerfile
* refactor dockerfile
* download correct gpg key
* Update badges in README.md
* fix typo in mask_rcnn_fpn_instance_segmentation.py
* update readme
* update README: add deeplab
* add deeplab test
* use PYTHON_VERSION for catkin_virtualenv
* add sample deeplab launch
* add deeplab semantic segmentation
* add mask rcnn in readme
* add gpu args in test
* add mask rcnn fpn tests
* chmod +x sample mask rcnn launch
* add sample mask_rcnn_fpn launch
* add mask_rcnn_fpn_instance_segmentation node
* support mask rcnn fpn
* fix models order
* support chainrcv 0.13 and chainer 6.0.0
* fix typo
* test with catkin_virtualenv 0.4.0
* use catkin_virtualenv 0.4.0
* fix typo
* fix for multi GPU usage
* Update README.md
* Update README.md
* disable faster_rcnn test
* Update README.md
* fix Dockerfile
* remove unused dockerfiles
* refactor cuda docker
* fix typo in docker files
* add cuda docker
* update cpu hooks
* add cpu docker
* update .travis.yml
* add kinetic docker file
* remove melodic nvidia dependencies
* Update README.md
* do not interactive build for melodic docker
* fix ros key server url
* install catkin_virtualenv 0.3.0 from source in docker
* add .travis.rosinstall
* use gpg2 for melodic key
* install gpg2 for docker
* fix typo in dockerfiles
* fix dockerfiles
* refactor melodic dockerfile
* fix docker files
* add melodic cuda92 docker
* add kinetic cuda100 docker
* add kinetic cuda92 docker
* add kinetic cuda91 docker
* add kinetic cuda90 docker
* add kinetic-cuda80 docker
* update melodic-cuda100 dockerfile
* add melodic + cuda10.0 nvidia dockerfile
* update medlodic docker file
* rm apt caches for docker build
* fix typo in Dockerfile
* add docker file
* test only for master branch
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* wait longer for test
* fix test
* fix test filename
* refactor test codes
* fix typo in test
* add node test
* fix time limit for tests
* set NOT_TEST_INSTALL=true
* update ssd300 test timeout
* update yolov3 test timeout
* do now test pspnet and ssd512 because of slow test
* fix typo in test_ssd512_object_detection.test
* wait 10minutes for slow test
* commentout slow test
* update travis test process
* wait for more time in test
* add pspnet install_pretrained_models
* update test
* Revert "update requirements"
  This reverts commit 9b0e33eb93fdd196fe560931a8c167e172c336d5.
* use catkin_virtualenv>=0.3.0
* update package.xml
* update requirements
* install pretrained model separately
* set test-limit for test
* update sample launch
* set jsk_tools as exec_depend
* add lt for nvidia-cuda version
* fix typo in .travis.yml
* use cuda9.1
* fix typo in CMakeLists
* update badge
* update test
* chmod for node_scripts
* add more dependencies
* add badge
* add travis
* add fcis
* fix typo in pspnet_semantic_segmentation
* add pspnet
* fix chmod
* refactor object_detection_node
* update CMakeLists.txt
* chmod node_scripts
* update README
* update requirements
* depend on nvidia-cuda
* add test_depend
* update README
* add install
* update readme
* update README
* add tests
* rename fix
* add ssd test
* add yolo
* refactor
* fix typo in fpn
* add fpn
* add faster_rcnn
* upda sample launches
* add object_detection.py
* update object_detection_node.py
* fix typo in package.xml
* update package.xml
* add sample_ssd_object_detection.launch
* initial commit
* Contributors: Shingo Kitagawa
