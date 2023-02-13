## Building and running tests

Make sure catkin is configured to build CISST tests:
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCISST_BUILD_TESTS=ON
```
Then clean and rebuild `saw_intuitive_research_kit_tests`.

To run tests:
```
~/catkin_ws/devel/bin/sawIntuitiveResearchKitTests --run
```
*Remember to replace `devel` with appropriate branch if you are not working off of devel.*
