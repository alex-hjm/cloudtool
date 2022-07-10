========================================================================
    CONSOLE APPLICATION : GetProfiles Project Overview
========================================================================

This is a simple application that returns profiles names of connected scanner.

You will learn how to:

* obtain profiles
* get actual profile
* set actual profile
* import/export profile
* create/delete profile
* update profile

How to build:

1. Copy GetProfiles folder to a location with Read and Write
   permissions (using the name <source>)
2. Open CMake
   2.1. Set Source code to <source>
   2.2. Set Binaries to <source>/_build or any other writable location
   2.3. Click Configure and Generate
3. Build project
4. Run PhoXiControl
   4.1. Connect to a scanner
5. Run GetProfiles application

The application will print out list of profiles of the connected scanner.
If not connected to any scanner, it will automatically connect to the first
in PhoXiControl.

/////////////////////////////////////////////////////////////////////////////
