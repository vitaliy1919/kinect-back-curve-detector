using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace KinectBackCurveDetector 
{
    class BackArea
    {
        private static Line2D leftShoulderLine;
        private static Line2D rightShoulderLine;
        private static Line2D hipLine;
        private static Line2D neckLine;
        
        private Point leftShoulder2D;
        private Point rightShoulder2D;
        private Point spineBase2D;
        private Point spineShoulder2D;
        private Point neck2D;

        private Line2D.PointPosition spineHipPosition;
        private Line2D.PointPosition spineNeckPosition;

        private Line2D.PointPosition leftShoulderRelativePosition;
        private Line2D.PointPosition rightShoulderRelativePosition;
        private Line2D.PointPosition hipRelativePosition;
        private Line2D.PointPosition neckRelativePosition;

        private static Point depthSpacePointToPoint(DepthSpacePoint depthSpacePoint)
        {
            return new Point(depthSpacePoint.X, depthSpacePoint.Y);
        }
        public BackArea(KinectSensor kinectSensor, IReadOnlyDictionary<JointType, Joint> joints)
        {
            var leftShoulder = joints[JointType.ShoulderLeft].Position;
            var rightShoulder = joints[JointType.ShoulderRight].Position;
            var spineBase = joints[JointType.SpineBase].Position;
            var spineShoulder = joints[JointType.SpineShoulder].Position;
            var neck = joints[JointType.Head].Position;
            var backPoints2D = new DepthSpacePoint[5];
            kinectSensor.CoordinateMapper.MapCameraPointsToDepthSpace(new CameraSpacePoint[5]
            { leftShoulder, rightShoulder, spineBase, spineShoulder, neck }, backPoints2D);
            leftShoulder2D = depthSpacePointToPoint(backPoints2D[0]);
            rightShoulder2D = depthSpacePointToPoint(backPoints2D[1]);
            spineBase2D = depthSpacePointToPoint(backPoints2D[2]);
            spineShoulder2D = depthSpacePointToPoint(backPoints2D[3]);
            neck2D = depthSpacePointToPoint(backPoints2D[4]);

            var spineLine = Line2D.makeLine(spineBase2D, spineShoulder2D);
            leftShoulderLine = spineLine.makeParalelLine(leftShoulder2D);
            rightShoulderLine = spineLine.makeParalelLine(rightShoulder2D);
            hipLine = spineLine.makePerpendicularLine(new Point(spineBase2D.X, spineBase2D.Y - 0.15 * (spineShoulder2D.Y - spineBase2D.Y)));
            neckLine = spineLine.makePerpendicularLine(neck2D);
            spineHipPosition = hipLine.GetPointPosition(spineShoulder2D);
            spineNeckPosition = neckLine.GetPointPosition(spineShoulder2D);
           
        }

        public bool isPointInSpineArea(Point point)
        {
            leftShoulderRelativePosition = leftShoulderLine.GetPointPosition(point);
            rightShoulderRelativePosition = rightShoulderLine.GetPointPosition(point);
            hipRelativePosition = hipLine.GetPointPosition(point);
            neckRelativePosition = neckLine.GetPointPosition(point);

            var pointIsOnBack =
                (leftShoulderRelativePosition == Line2D.PointPosition.Right || leftShoulderRelativePosition == Line2D.PointPosition.OnLine) &&
                (rightShoulderRelativePosition == Line2D.PointPosition.Left || rightShoulderRelativePosition == Line2D.PointPosition.OnLine) &&
                (hipRelativePosition == spineHipPosition || hipRelativePosition == Line2D.PointPosition.OnLine) &&
                (neckRelativePosition == spineNeckPosition || neckRelativePosition == Line2D.PointPosition.OnLine);
            return pointIsOnBack;
        }
    }
}
