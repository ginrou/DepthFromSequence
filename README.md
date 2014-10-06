## Depth From Sequence
Handy depth estimation software from image sequences.

This software is intended to port [LensBlur of Android](http://googleresearch.blogspot.jp/2014/04/lens-blur-in-new-google-camera-app.html) to iOS.
But, not only in iOS, it can be used in any device, platform which OpenCV and C++ can use.


![Whats this](https://raw.githubusercontent.com/wiki/ginrou/DepthFromSequence/image/whats_this.gif)

##### Input
Multiple images taken in a sequence.
![input images](https://raw.githubusercontent.com/wiki/ginrou/DepthFromSequence/image/input.gif)


##### Output
Depth map of the scene.
![output depth map](https://raw.githubusercontent.com/wiki/ginrou/DepthFromSequence/image/depth_smooth.png)


#### 日本語
日本語の解説はこちらから(後ほど書きます)


### Supported Platform and Dependencies
* Mac OSX (Probablly Linux too)
 * CMake, OpenCV
* iOS
 * CocoaPods


### Trial Use
When you want to try this software, try with sample CLI software.

```
$ git clone git@github.com:ginrou/DepthFromSequence.git
$ cd sample
$ cmake .
$ ./main.out ./data/*.jpg
```
Raw depth map, smooth depth map, and color depth map is outputed.
You can test with your own image sequence by changing input arguments.

### Future Work
* Performace Improvement.
* Clear Licence.
