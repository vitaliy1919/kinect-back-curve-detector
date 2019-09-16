# Kinect Back Curve Detector 
## Overview 

The application uses body and depth data from Kinect V 2.0 to create an outline of person's back in outline.

In order for the application to work correctly, a person has to sit with their back oriented towards a Kinect, the frontal plane has to be approximately parallel to the Kinect camera plane.


![Program Main Window](report-images/mainwindow.png)

## Algorithm overview

![Planes overview](report-images/planes.png)

At first, all 2D points of the body (acquired using Kinect body index data source) are converted to 3D points using Kinect coordinate mapper. After that these 3D points are projected to the median plane of the body, thus getting the view of the back from the side.

Now, in order to find the outline of the back one needs to pick the extreme points from the projected points.

Extreme points:
![Extreme points](report-images/extreme_points.png)

(*green - extreme points, blue - projected points*)

However, the data acquired from Kinect contains noise. The strategy of picking extreme points doesn't work on such data, because one may choose noisy point as a back point that spoils the outline quite drastically.

Noisy data:

![Noisy data](report-images/noisy_data.png)

(*green - extreme points on noisy data*)

Hence at first the data needs to be filtered. The problem of detecting noisy points in point clouds (we consider the projected points as a point cloud) is quite time consuming. The common approach of solving such a problem is k-nearest neighbors. The main idea is that the k nearest point to each point of a point cloud, then calculate some metrics with regards to them (usually the metrics is related to the sum of distances (not necessary euclidean distances) from the point to its neighbors. Then the threshold is set and all points with the metrics greater than the threshold are considered noisy.

This approach works fine, however, has one crucial disadvantage - it really slow for our purposes. Even with KD-tree that is considered to be one of the fastest way of finding neighbors, the computing time for one frame way around 330 - 600 ms (depending on a value of k and the number of points that vary from frame to frame). This means that only 1.5 - 3 frames will be proceed per second, while Kinect is sending us 30 frames. 

Thus, another algorithm needs to be used. Luckily, the data obtained from Kinect isn't just a random point cloud. It turned out that it is possible to change our problem. So, instead of finding noisy points in the whole point cloud, it is split in rows, then the noisy points are found in each row. Rows are chosen based on rows of pixels in the depth frame, so each row of pixels is converted to 3D row of points, then we use k-nearest neighbors to determine the noisy points (now it is easy to find nearest points - after the row is sorted, previous k / 2 and next k / 2 neighbors are actually these k nearest neighbors). It turned out that performance of this simple algorithm is very good - 21 - 28 ms in debug mode and 9 - 11 ms in Release mode (run without debugger attached, CPU - i7-4790, 16 Gb of DDR3 memory and integrated video card). This means that all frames from Kinect are processed and there is this time to do something with the back data.

Rows visualisation, each row is in different color:

![Rows visualization](report-images/row_visualization.png)

## Back data processing

Right now the back data received from Kinect is not processed, it's just shown on the screen.

if you want to actually process data, you need to do it in the following function:

```MainWindow.xaml.cs```:
``` csharp
private void MultisorceReader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
{
    ...

    // calculate and show the back position on the screen
    var spinePoints = GetSpinePoints(body, depthBuffer, bodyIndexBuffer);
    
    // Process the spinePoints here

    drawSpinePoints(spinePoints);
    RenderBodyIndexPixels();

    ...
}
```
```spinePoints``` is the ```List``` of ```Point```, where the first coordinate of point stores depth, the second - height relative to Kinect sensor. The List is sorted by height, meaning that the first point has the maximum height, the last one - minimum.