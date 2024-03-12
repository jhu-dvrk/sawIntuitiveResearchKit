# Introduction

This directory contains an example of a C++ cisst/SAW component derived from the default dVRK PSM component.  It can be used as a template for any dVRK users who wishes to implement a new PSM behavior for their research.

There are a few advantages and drawbacks to this approach:
* Pros:
  * Since this example uses the *cisstMultiTask* framework (see https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts), the component will run in the same process as the dVRK controller and communication will be very fast (as opposed to messages passed over a middleware such as ROS).
  * Using a component derived from an existing one, the user can keep some of the existing features and add/replace just what is need.
  * Since the example extends an existing component, the existing interfaces (ROS, Qt GUI) from the base class will still be available.
* Cons:
  * The code is in C++ and uses the cisst libraries so users will need to get familiarized with *cisstVector*, *cisstMultiTask* and *cisstParameterTypes* (see https://github.com/jhu-cisst/cisst/wiki).
  * As for any C++ code, the user will have to write/update some CMake scripts.  Since the dVRK build relies on ROS catkin build tools, the user will also have to write/update a `package.xml` file.
  * Since the component is just a computing component, the user has to figure out how to interact with it.  This example also provides an example of ROS bridge and Qt GUI.  This applies to all new "features" so if the code relies on the base class, the dVRK console will still expose the features of the base class.

It is recommended to copy/paste the content of this directory in a different location in your catkin workspace (under `src`).  Then you will need to rename the ROS package name and library name to avoid conflicts when you're building the code.  You should also make sure your new directory is under some kind of [VCS](https://en.wikipedia.org/wiki/Version_control) (git, mercurial, svn, cvs...).

# CMake and catkin

This step requires some simple query/replace to make sure your new component/package doesn't conflict with the existing example.  The two files to modify are located in the `components` directory.

## `package.xml`

In `package.xml` query/replace the string `saw_intuitive_research_kit_example_derived_psm` by whatever you want to call your package.  For example, `my_package`.https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.1/dvrk-console.html

## `CMakeLists.txt`

You should be able to use the `CMakeLists.txt` as-is but you might need to update it if you have some other external dependencies or more C++ files.  Ideally, you should also rename the library to avoid potential conflicts (query/replace `sawIntuitiveResearchKitDerivedPSM`).

# Code

The code is located under `components` with the header file (`.h`) in `components/include` and the implementation files (`.cpp`) are in `components/code`.  The example code contains some comments to help understand how to write your own code.

# Console configuration file

Once you have created your new component, you will need to use it.  To do so, you can edit or create a new `main.cpp`, create the component (C++ `new`), add it to the cisst component manager (C++ `manager->AddComponent(...)`), configure it and connect it to the MTM and PSM components as well as the ROS bridge and/or Qt Widget.  You will also need to create a new `CMakeLists.txt` to compile your new code.

Alternatively, the whole process can also be done using a single configuration file.  The main two dVRK applications, `sawIntuitiveResearchKitQtConsoleJSON` (non ROS) and `dvrk_console_json` (ROS node), use the dVRK console JSON configuration file.  The format is described in https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.1/dvrk-console.html.  You can find examples of console configuration files that show how to use your newly created PSM component instead of the default one in the `share` directory.  For example, in `share/jhu-dVRK/console-MTML-PSM1Derived-Teleop.json`:
* `"component-manager"` / `"components"`: defines which library and type to use to create the "PSM1" PSM component
* `"arm"` / `"type"`: tells the console to not create a PSM component using the default class.  Instead the code will now look for an existing component with the name "PSM1", i.e. the one we just created in the `"component-manager"` section.

If you have renamed the library and/or class name in your `CMakeLists.txt` and C++ code, make sure you also update the `"shared-library"` and `"class-name"` fields in the console configuration file.
