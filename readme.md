## Features

- Emulation of a little part of arduino library (most used functions) to allow proper debugging.
- Basic use : just put the files "settings.h", "arduino.h", and "main.cpp" in your project folder and add "main.cpp" to the sources, without any code modification. 
- Cross platform and ide (compile with -pedantic)
- Very few reserved keywords other than the emulated arduino api : `main`, `myArd`,  and `TWO_INCLUDE_DIRECT_COMPILATION_MODE`
- Allows plugging an external source with its voltage/impedence(real only) couple. Behavior of the circuit is "simulated" (poorly) with only passive components + internal protection diode (two slope like impedance) emulated, with noise. Basic survival checks are automatically performed, in worst case.
- Real emulation of "millis()" call
- Undefining TWO_INCLUDE_DIRECT_COMPILATION_MODE by uncommenting the line
```cpp
//#undef TWO_INCLUDE_DIRECT_COMPILATION_MODE
```
to
```cpp
#undef TWO_INCLUDE_DIRECT_COMPILATION_MODE
```
 in the "setting.h" file will trigger a call to "altererDefArduino" (a function to be user-implemented)  at every call to the api and every 200ms. Its prototype :
`void altererDefArduino(myArd::arduino&, myArd::alterationCallReason);`
This function will never be called again before it returns.
Also, the counter of "millis()" is automatically stuck while this function is performing.
However it is not during the pause triggered by the debugger or the breakpoints.
- `std::array <bool, 10> breakingTicks;` allows to break on some api calls only.
- When an api breaks, you can easily find the returned variable with their name, being :
"dbtbm_xx", and modify them before their return.

- A lot (even more) of missing functions. The backbone is here, so any contributer would be welcome.


####To do list (for the author or any brave contributer. I wish you good luck if you try to read this)
-add pause and unpause before and after every call to breakAndAlter (en prenant garde à ne pas l'appeler deux fois de suite)
- Detail the readme and add a example of alteration function with modification of the breaking behavior
- add a boolean that allows swithcing from "worst" mode to "random" mode for the noise direction.
- Extracting constants 
- changing callbackInterval to a modifiable so its modification is possible at compilation and run-time.
- In the function breakAndAlter, 