rm -r build
rm -r devel
mkdir reports
catkin config --cmake-args -DCMAKE_C_FLAGS=-fprofile-arcs -DCMAKE_C_FLAGS=-ftest-coverage -DCMAKE_EXE_LINKER_FLAGS=-fprofile-arcs -DCMAKE_EXE_LINKER_FLAGS=-ftest-coverage -DCMAKE_BUILD_TYPE=Coverage
catkin build
catkin run_tests
gcovr -r ./src/catmull_ros/include ./src/catmull_ros/src ./build -x -o  reports/gcovr-report.xml
gcovr -r ./src/catmull_ros/include ./src/catmull_ros/src ./build -b -x -o reports/gcovr-report.b.xml
cppcheck --xml --xml-version=2 --enable=all src/catmull_ros/include/catmull_ros/*.hpp 2> reports/cppcheck-report.include.xml
cppcheck-htmlreport --source-dir=. --report-dir=reports/html/include --file=reports/cppcheck-report.include.xml
firefox reports/html/include/index.html
cppcheck --xml --xml-version=2 --enable=all src/catmull_ros/test/*.cpp 2> reports/cppcheck-report.test.xml
cppcheck-htmlreport --source-dir=. --report-dir=reports/html/test --file=reports/cppcheck-report.test.xml
firefox reports/html/test/index.html
~/sonar-scanner-3.2.0.1227-linux/bin/sonar-scanner
rm -r build
rm -r devel
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
catkin run_tests
