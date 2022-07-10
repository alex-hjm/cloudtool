========================================================================
    CONSOLE APPLICATION : RotatedCalibration Project Overview
========================================================================

This example shows how to "calibrate" the scanner with the rotary table.

You will learn how to:

* obtain transformations from marker space to camera space,
* apply custom transformation matrix before triggering the scan.

Note: this example requires API version 1.2+

The scenario is that we want to use rotary table to create a 3D model of some
object using PhoXi 3D scanner. The problem is that this object covers the whole
marker plate that is usually used for this task. (see Scan alignment on
Photoneo Wiki -  http://wiki.photoneo.com/index.php/Scans_alignment).

As a solution, we will use marker plate to record coordinate transformations
for several rotations of the rotary table. Then we put the object on the rotary
table and use recorded transformations when taking the 3D scans. As a result,
all scans will have the same coordinate system and are ready for merging.

The process could be as follows:
1) Put a marker plate on the rotary table. 
2) Fix the scanner so as it does not move with respect to the centre of the rotary table.
3) Set the recognition of marker plate in PhoXi scanner.
4) Rotate the table to one of desired positions, trigger the scan and record transformation matrix.
5) Repeat step 4 until transformations for all desired rotations are acquired.
6) Put the object on the rotary table.
7) Rotate the table to one of recorded positions, apply the corresponding transformation matrix, trigger and save the scan.
8) Repeat step 7 until scans at all rotations are captured.


/////////////////////////////////////////////////////////////////////////////