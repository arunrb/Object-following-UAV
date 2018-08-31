# Object-following-UAV
The Parrot AR.Drone 2 uses the onboard camera to track and follow the selected object in an indoor environment. The tracking is based on CAMSHIFT algorithm, implemented in C++ using the OpenCV library.

NOOBS - BEFORE YOU BUILD:
 - You should install Microsoft Visual Studio Community 2015.
     To download VS, https://www.microsoft.com/en-us/download/details.aspx?id=48146

 - CV Drone supports VC++ 2015.

LIBRARY DEPENDENCIES
  CV Drone uses following libraries.
  - OpenCV 3.1.0 <3-clause BSD license>
    http://opencv.org/
  - FFmpeg 2.2.3 <LGPL v2.1 license>
    http://www.ffmpeg.org/
  
GENERAL PROCEDURE TO RUN THE CODE:

	Without Drone ( for checking the workability of code) 

1) Extract the Camshift_object_tracking.zip file in any directory
2) Open .\camshift_object_tracking\build\vs2015\Camshift_object_tracking.sln
3) Press Ctrl+Shift+B to build.
4) Press F5 to run the program. 
	
	With drone connected

1) Extract the Camshift_object_tracking.zip file in any directory
2) Open .\camshift_object_tracking\build\vs2015\Camshift_object_tracking.sln
3) Press Ctrl+Shift+B to build.
4) connect the AR drone 2.0.
5) Press F5 to run the program. 
