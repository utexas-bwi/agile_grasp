^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agile_grasp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.3 (2018-03-09)
------------------
* install SVM data files (`utexas-bwi/bwi#54 <https://github.com/utexas-bwi/bwi/issues/54>`_)
* Contributors: Jack O'Quin

0.8.2 (2018-03-08)
------------------
* fix add_dependencies error (`utexas-bwi/bwi#54 <https://github.com/utexas-bwi/bwi/issues/54>`_)
* Contributors: Jack O'Quin

0.8.1 (2018-03-08)
------------------
* make me maintainer of this fork (`utexas-bwi/bwi#54 <https://github.com/utexas-bwi/bwi/issues/54>`_)
* add message dependency for grasp_localizer (`utexas-bwi/bwi#54 <https://github.com/utexas-bwi/bwi/issues/54>`_)
* switch dependency from OpenCV2 to OpenCV3 (`utexas-bwi/bwi#54 <https://github.com/utexas-bwi/bwi/issues/54>`_)
* Contributors: Jack O'Quin

0.8.0 (2018-03-03)
------------------
* Custom UTexas-BWI version for Kinetic
* Fix SVM loading
* Initialize camera transforms in hand search process
* Update svm code for OpenCV3
  Minimum changes required to compile under OpenCV3
* Merge branch 'master' into kinetic
* Migrate some deprecated OpenCV, Eigen usage
  Use new CV SVM interface
  Use Eigen3 cmake package
  Eigen deprecated sum on bool vectors for clarity
  http://eigen.tuxfamily.org/bz/show_bug.cgi?id=426
* Kinetic: add explicit dependency on libopencv-dev (`#5 <https://github.com/utexas-bwi/agile_grasp/issues/5>`_)
* Kinetic: force use of OpenCV2 interfaces (`#5 <https://github.com/utexas-bwi/agile_grasp/issues/5>`_)
* Kinetic: migration from Indigo Eigen interface (`#5 <https://github.com/utexas-bwi/agile_grasp/issues/5>`_)
* added optional parameter
* launch file for hsrb
* added additional grasp information in the srv response and a launch file for the hsrb
* added service for grasp detection
* handles and hands are now published on separate topics
* simplify Readme
* Contributors: Jack O'Quin, Jivko Sinapov, Max Svetlik, Nick Walker, atp

0.7.2 (2015-05-16)
------------------
* added author to package.xml

0.7.1 (2015-05-16)
------------------
* added dependency for visualization msgs
* Contributors: Andreas

0.7.0 (2015-05-13)
------------------
* update CMakeLists
* added rviz visualization
* fixed instructions -> (6)
* added instructions
* fixed launch files
* Contributors: atp

* added rviz visualization
* fixed instructions -> (6)
* added instructions
* fixed launch files
* Contributors: atp

0.6.2 (2015-04-11)
------------------
* fix some dependencies
* Contributors: atp

0.6.1 (2015-04-10)
------------------
* initial commit
* Contributors: atp
