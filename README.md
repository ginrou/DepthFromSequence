## Depth From Sequence
Handy depth estimation softwear from image sequences.

![Whats this](https://raw.githubusercontent.com/ginrou/DepthFromSequence/master/whats_this.gif)

### System Requierments
* CMake and OpenCV
 * Mac OSX, Linux(probably)
* iOS

### How To Use

```
$ git clone git@github.com:ginrou/DepthFromSequence.git
$ cd sample
$ cmake .
$ ./main.out ./data/*.jpg
```

Then, raw depth map, smooth depth map, and color depth map is outputed.