cppcheck --xml --xml-version=2 --enable=all include/catmull_ros/*.hpp test 2> ../../build/cppcheck-report.xml
~/sonar-scanner-3.2.0.1227-linux/bin/sonar-scanner
