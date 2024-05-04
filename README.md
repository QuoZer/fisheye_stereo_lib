# Orthogonal Fisheye Stereo library

A C++ library for fisheye lens ROI distortion removal. Provides a class for fisheye stereo systems that can rectify images, compute disparity maps and depth maps. The distortion can be removed in any part of the image allowing to e.g. perform stereo matching with cameras pointing in different directions.

![4pic_example](https://github.com/QuoZer/fy_lib_source/assets/57410392/028f6c62-9bed-4eb7-8c66-98c6d28a7224)

Supported camera models:
- Pinhole
- Scaramuzza's camera model
- Kannala-Brandt model
- Mei model
  
Custom camera models can be added (see below).

## Requirements 
- OpenCV

## How to use

The 'examples' folder contains some demos. The main steps are:
1. Create a `SurroundSystem` object.
2. Load the stereo system parameters from a file or define them in code.
3. Precompute the lookup tables.
4. Use the 'getImage' method with the desired output type ('RECTIFIED', 'DISPARITY' or 'DEPTH') to process images.

To use custom camera models, you can create a new class that inherits from `CameraModel` and implement the `projectWorldToPixel` and `projectPixelToWorld` methods.

## Parameters 

The stereo system parameters are defined in an XML file. The following parameters are required:

 - Camera models.
 - Intrinsic parameters of the cameras (focal length, principal point, distortion coefficients).
 - Extrinsic parameters of the cameras (rotation and translation).
 - Stereopair definition (which cameras to use, view direction, output image size, stereo matcher parameters).

For the stereo matcher parameters refer to the OpenCV documentation.


