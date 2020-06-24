# Self contained example using the Isaac C API

Contains three parts:
1. `c_api_example.c` - an example app that uses the Isaac C API
2. `c_api_example.app.json` - creates an Isaac node that will echo any message
      received in the "in" channel back via the "out" channel
3. `build.sh` - a script that will build the example using gcc.

To use the sample app, copy all files to your Isaac C API deploy directory,
then run from the Isaac C API directory run `./build.sh`.
