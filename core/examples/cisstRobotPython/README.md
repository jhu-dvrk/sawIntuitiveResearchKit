
# Using cisstRobotPython to replicate the forward and inverse kinematics of the dVRK

## Compilation and environment

By default the Python wrapper for the cisst libraries are compiled when using `catkin build`.  If you compiled the code using `CMake` directly, you need to make sure the `SWIG_PYTHON` flags are properly set when compiling *cisst*.

To set your Python path so Python can find *cisstRobotPython*, you can use the script `cisstvars.sh`.  When using `catkin`:
```sh 
source ~/catkin_ws/devel/cisstvars.sh
```

We provide a basic example.  To run it, make sure you are in this directory and type:
```sh
python3 example.py
```

## Main differences with the dVRK C++

* *cisstRobotPython* can't load `.json` files directly, instead you will have to use `.rob` files (some examples can be found in directory `share/deprecated`)
* For the PSM, the dVRK 2.0 uses two DH files, one for the first 3 joints and one for the instrument.  This can be achieved in Python by loading two separate `.rob` files consecutively.  The `.rob` files don't exist so you will have to create your own from the `psm.json` and the `<instrument>.json` files (see directories `share/tool` and `share/`)
* `base-frame` can't be defined in `.rob` file but you can add one in Python (see below)
* Starting with the dVRK 2.0, the ECM inverse kinematics uses a closed form computation and this is not available through Python

## Troubleshooting and tips

* Failed to import `cisstRobotPython`, error looks like:
    ```python
    ImportError: dynamic module does not define init function (init_cisstRobotPython)
    ```
  This can occur if the Python wrappers were compiled for Python 3 but you're using Python 2.  You can either change your Python interpreter or recompile the wrappers for Python 2.  To do so, run CMake in the cisst build tree (`ccmake ~/catkin_ws/build/cisst` and change all the references to Python 3 to Python 2).  Then re-compile your catkin workspace with `catkin build`.

* Check the return value of `LoadRobot`.
    ```python
    r = cisstRobotPython.robManipulator()
    result = r.LoadRobot('/home/cstar/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob')
    # result should be 0 if successfully loaded
    # one can also make sure the number of links is correct
    print(r.links.size())
    ```

* To modify the base frame, assign a 4x4 matrix to `r.Rtw0`

* To add a tool tip transform, create a dummy manipulator and attach it to the existing manipulator:
    ```python
    tt = robManipulator()
    tt.Rtw0 = tooltip
    r.Attach(tt)
    ```

* Accessing and modifying DH parameters:
    ```python
    l2 = r.links[2]
    k2 = l2.GetKinematics()
    k2offset = k2.PositionOffset()
    k2.SetPositionOffset(k2offset + some_noise)
    ```

* Calculating the Jacobian:
    ```python
    jp = np.zeros(6) # joint angles
    J_body = np.zeros(shape=(6,6), dtype=np.double) # placeholder
    r.JacobianBody(jp, J_body)
    print(J_body)
    ```
