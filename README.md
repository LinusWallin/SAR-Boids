# SAR-Boids

## Build Requirements
- Windows (Can be changed by modifying the osqp_wrapper.c file)
- [Unity](https://unity.com/)
- [OSQP](https://github.com/osqp)
- [MSYS2 MinGW64](https://www.msys2.org/)

### Build
1. Clone the [OSQP](https://github.com/osqp) repository.
2. Make sure that there is a osqp.dll file in the Assets/Plugins folder and make sure it is up to date.
3. Open the MSYS2 MinGW64 terminal in the osqp folder and run the following commands:
    - `mkdir build`
    - `cd build`
    - `cmake -G “MinGW MakeFiles” ..`
    - `mingw32-make`
4. Build the dll wrapper file by running the following command in a MSYS2 MinGW64 terminal in the Assets/Plugins folder:
    - `gcc -shared -o osqp_wrapper.dll osqp_wrapper.c -I../osqp/include/public -I../osqp/build_api/include/public ../osqp/build_api/out/libosqpstatic.a`
5. Open Unity and play the scene.

## Modifying the Simulation

### Parameters
The easiest way to modify the behavior of the simulation is by changing the parameters used in the simulation. This can be done my opening the boidSettings.cs file (located in `Assets\Scripts\boidSettings.cs`) in the Unity UI. Most of the parameters can be changed without any constraints, but there are certain parameters that are more sensitive and could cause some issues in the current version of the implementation. These parameters are listed below and the recommended steps to change them:
- The `Grid Size` parameter should match the distance between the planes that define the space in which the boids are allowed to exist, but they also need to divisible by the `Cell Radius` * 2.
- It's important that the `Start Position` is inside of the grid and preferably at the beginning of the grid to make sure that the boids are able to spawn in.
- The `Separation Ratio` parameter defines how close the aids to navigation (ATONs) spawn to each other. For performance this number is kept higher, since it leads to a larger number of calculations as the ratio is reduced. TLDR: ATONs are the boids which spawn around obstacles and on the planes to guide the boids, but their speed and direction are always 0. 

### Creating New Scenarios
Creating new scenarios is quite simple. All you need to do is to create a new scene and copy over the `BoundingBox`, `Simulation`, and `TargetLocation` game objects from the MainScene to your new scene. You can then add new obstacles to the scene by creating a 3D object (I have not tried round objects, so there could be issues with those) and changing the **Layer** of the new game object to `Obstacle`.