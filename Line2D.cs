using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
namespace Microsoft.Samples.Kinect.BodyIndexBasics
{
    class Line2D
    {
        private double lean;
        private double coef;
        private bool isVertical;
        private Point top, bottom;
        private static double cross(Point a, Point b)
        {
            return a.X * b.Y - a.Y * b.X;
        }
        private static double signed_area_parallelogram(Point p1, Point p2, Point p3)
        {
            return cross(new Point(p2.X - p1.X, p2.Y - p1.Y), new Point(p3.X - p2.X, p3.Y - p2.Y));
        }

        private static double triangle_area(Point p1, Point p2, Point p3)
        {
            return Math.Abs(signed_area_parallelogram(p1, p2, p3)) / 2.0;
        }

        private static bool clockwise(Point p1, Point p2, Point p3)
        {
            return signed_area_parallelogram(p1, p2, p3) < 0;
        }

        private static bool counter_clockwise(Point p1, Point p2, Point p3)
        {
            return signed_area_parallelogram(p1, p2, p3) > 0;
        }
        private static int compareDouble(double a, double b, double eps = 1e-6)
        {
            if (Math.Abs(a - b) < eps)
                return 0;
            return a.CompareTo(b);
        }
        public double getX(double y)
        {
            if (isVertical)
                return coef;
            if (compareDouble(lean, 0) == 0)
                throw new ArgumentException("Lean is 0");
            return (y - coef) / lean;
        }

        public double getY(double x)
        {
            if (isVertical)
                throw new ArgumentException("Line is vertical");
            return x * lean + coef;
        }
        public Line2D makeParalelLine(Point a)
        {
            Line2D line = new Line2D();
            if (isVertical)
            {
                line.isVertical = true;
                line.coef = a.X;
                line.setPoints(a);
                return line;
            }
            line.lean = lean;
            line.coef = a.Y - line.lean * a.X;
            line.setPoints(a);
            return line;

        }

        private void setPoints(Point a)
        {
            if (isVertical)
            {
                top = a;
                bottom = new Point(a.X - 10, a.Y);
                return;
            }
            if (compareDouble(lean, 0) == 0)
            {
                top = a;
                bottom = new Point(a.X, a.Y - 10);
                return;
            }
            Point point = new Point(getX(a.Y - 10), a.Y - 10);
            if (a.Y > point.Y)
            {
                top = a;
                bottom = point;
            }
            else
            {
                top = point;
                bottom = a;
            }
        }
        public Line2D makePerpendicularLine(Point a)
        {
            Line2D line = new Line2D();
            if (isVertical)
            {
                line.lean = 0;
                line.coef = a.Y;
                line.setPoints(a);
                return line;
            }

            if (compareDouble(lean, 0) == 0)
            {
                line.isVertical = true;
                line.coef = a.X;
                line.setPoints(a);
                return line;
            }

            line.lean = -1 / lean;
            line.coef = a.Y - lean * a.X;
            line.setPoints(a);
            return line;
        }
        public static Line2D makeLine(Point a, Point b)
        {
            Line2D line = new Line2D();
            if (a.Y > b.Y)
            {
                line.top = a;
                line.bottom = b;
            } 
            else
            {
                line.top = b;
                line.bottom = a;
            }
            if (compareDouble(a.X, b.X) == 0)
            {
                line.isVertical = true;
                line.coef = a.X;
                return line;
            }
            line.lean = (b.Y - a.Y) / (b.X - a.X);
            line.coef = a.X * line.lean;
            
            return line;
        }

        public enum PointPosition
        {
            Left,
            Right,
            OnLine
        }

        public PointPosition GetPointPosition(Point p)
        {
            if (compareDouble(getX(p.Y), p.X) == 0)
                return PointPosition.OnLine;
            double area = signed_area_parallelogram(bottom, top, p);
            if (area > 0)
                return PointPosition.Left;
            return PointPosition.Right;
        }
    }
}
