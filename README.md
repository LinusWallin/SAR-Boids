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