========================================================================
    CONSOLE APPLICATION : MinimalPclExample Project Overview
========================================================================

This is a simple example of how to find and connect an available PhoXi Device
and then convert point clouds from Photoneo format to PCL format.

You will learn how to:

* use Point Cloud Library in your project with PhoXi API,
* convert scanned frame into PCL format.

The project must  be correctly set up. In VisualStudio 2015 (for msvc14) or
VisualStudio 2013 (for msvc12):

Right click on your project → Properties → C/C++ → General → Additional Include
Directories:

  - C:\Program Files\PhotoneoPhoXiControl\API\include
    - This is where our API headers are.
  - <install-dir>\pcl-<ver>\include\pcl-<ver>
    - You should find another pcl folder containing folders (2d, common,..)
      and header files (cloud_iterator.h, conversions.h,...) in this directory.
  - <install-dir>\eigen-<ver>
    - This directory should contain Eigen folder which has a src folder and
      files without extensions.
  - <install-dir>\boost-<ver>
    - This directory should contain a boost folder containing all the headers.

Linker → General → Additional Library Directories:
  - C:\Program Files\PhotoneoPhoXiControl\API\lib
  - <install-dir>\pcl-<ver>\lib
  - <install-dir>\boost-<ver>\lib64-msvc-14.0

Linker → Input → Additional Dependencies:
  - PhoXi_API_msvc14_Release_1.2.6.lib
  - Pcl_common_release.lib

<install-dir>	is a stand-in for path to a directory where PCL is installed
             	on your computer.
<ver>			is a stand-in for the version number of PCL you are going to use

/////////////////////////////////////////////////////////////////////////////
