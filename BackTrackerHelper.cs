using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using Microsoft.Kinect;

namespace KinectBackCurveDetector
{
    class BackTrackerHelper
    {

        public static void calculateRotationAngle(CameraSpacePoint leftShoulder, CameraSpacePoint rightShoulder, out double tan, out double cos)
        {
            tan = 0;
            cos = 1;

            var zDiff = rightShoulder.Z - leftShoulder.Z;
            var xDiff = rightShoulder.X - leftShoulder.X;
            tan = -zDiff / xDiff;
            cos = xDiff / (Math.Sqrt(xDiff * xDiff + zDiff * zDiff));
            //if (Math.Abs(this.angle) > 1e-5)
            //{
            //    var alpha = this.angle / 360f * 2 * Math.PI;
            //    tan = Math.Tan(alpha);
            //    cos = Math.Cos(alpha);
            //}
        }

        // calculate spine point from the sorted row of points
        // spine point is the leftmost point from the row
        // the only problem is that the data is noisy
        // hence at first data is filtered
        // using the modified k-nearest neighbors algorithm
        // the distance to the k (k = 6) left points is calculated
        // the threshold is 0.005
        public static Point calculateSpinePoint(List<Point> row, bool adjustForKinectAngle = true)
        {
            int adjacentPointNumber = 12;

            for (int i = 0; i < row.Count; i++)
            {
                int realAdjacentPointCount = 0;
                double distanceMetrics = 0.0;
                for (int iter = 0; iter < (adjacentPointNumber / 2); iter++)
                {
                    if (iter + i < row.Count)
                    {
                        distanceMetrics += Math.Abs((double)(row[i].X - row[i + iter].X));
                        realAdjacentPointCount++;
                    }
                }
                double a = -(14 * Math.PI) / 180.0;
                double sin = Math.Sin(a);
                double cos = Math.Cos(a);
                var rotatedPoint = new Point(
                        row[i].X * cos + row[i].Y * sin,
                        -row[i].X * sin + row[i].Y * cos);
                double updatedMetrics = (realAdjacentPointCount == 0) ? 0.0 : (distanceMetrics / realAdjacentPointCount);
                if (updatedMetrics < 0.005)
                {
                    if (adjustForKinectAngle)
                        return rotatedPoint;
                    else
                        return row[i];
                }
            }
            throw new Exception("No spine points found");
        }

        public static bool isBodyTracked(IReadOnlyDictionary<JointType, Joint> joints)
        {
            var backTracked =
                joints[JointType.SpineShoulder].TrackingState == TrackingState.Tracked &&
                joints[JointType.SpineBase].TrackingState == TrackingState.Tracked &&
                joints[JointType.ShoulderLeft].TrackingState == TrackingState.Tracked &&
                joints[JointType.ShoulderRight].TrackingState == TrackingState.Tracked &&
                joints[JointType.Head].TrackingState == TrackingState.Tracked;
            return backTracked;
        }

        // calculate upper and lower bound to a given accuracy
        // e.g if accuracy is 0.4 
        // upper bound: 4.3 -> 4.4, 4.4 -> 4.4, 4.5 -> 5.0
        // lower bound: 4.4 -> 4.0, 4.4 -> 4.4, 4.5 -> 4.4
        public static double upperBound(double number, double acc)
        {
            double integerPart = Math.Truncate(number);
            double floatPart = number - integerPart;
            if (floatPart < acc || Math.Abs(floatPart - acc) < 1e-5)
                return integerPart + acc;
            return integerPart + 1;
        }

        public static double lowerBound(double number, double acc)
        {
            double integerPart = Math.Truncate(number);
            double floatPart = number - integerPart;
            if (floatPart > acc || Math.Abs(floatPart - acc) < 1e-5)
                return integerPart - acc;
            return integerPart - 1;
        }
    }
}
