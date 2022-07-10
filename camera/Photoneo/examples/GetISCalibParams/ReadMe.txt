========================================================================
    CONSOLE APPLICATION : GetISCalibParams Project Overview
========================================================================

This is a simple application that returns image sensor calibration parameters
of connected scanner.

You will learn how to:

* obtain image sensor calibration parameters

How to build:

1. Copy GetISCalibParams_CPP folder to a location with Read and Write
   permissions (using the name <source>)
2. Open CMake
   2.1. Set Source code to <source>
   2.2. Set Binaries to <source>/_build or any other writable location
   2.3. Click Configure and Generate
3. Build project
4. Run PhoXiControl
   4.1. Connect to a scanner
5. Run GetISCalibParams application

The application will print out calibration params of the connected scanner.
If not connected to any scanner, it will automatically connect to the first
in PhoXiControl.

/////////////////////////////////////////////////////////////////////////////
