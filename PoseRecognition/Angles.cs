using System;
using System.Windows.Media.Media3D;

namespace PoseRecognition
{
    class Angles
    {
        // Calculates the 3D angle produced by three points a, b and c
        // Each of them must be 3D, thus composed by x, y and z
        // The angle which is calculated is BAC, thus the second point is always used as vertex
        // Returns null if any of the three points have missing data
        public static double? Calculate3DAngle(Point3D? a, Point3D? b, Point3D? c)
        {
            if (a == null || b == null || c == null)
                return null;

            // Calculate AB and BC vectors
            Vector3D ab = new Vector3D { X = a.Value.X - b.Value.X, Y = a.Value.Y - b.Value.Y, Z = a.Value.Z - b.Value.Z };
            Vector3D bc = new Vector3D { X = c.Value.X - b.Value.X, Y = c.Value.Y - b.Value.Y, Z = c.Value.Z - b.Value.Z };

            // Normalize AB and BC:
            double abMag = Math.Sqrt(Math.Pow(ab.X, 2) + Math.Pow(ab.Y, 2) + Math.Pow(ab.Z, 2));
            Vector3D abNorm = new Vector3D { X = ab.X / abMag, Y = ab.Y / abMag, Z = ab.Z / abMag };

            double bcMag = Math.Sqrt(Math.Pow(bc.X, 2) + Math.Pow(bc.Y, 2) + Math.Pow(bc.Z, 2));
            Vector3D bcNorm = new Vector3D { X = bc.X / bcMag, Y = bc.Y / bcMag, Z = bc.Z / bcMag };

            // Calculate the dot product and then the arcsin to obtain the angle in radiants
            double rad = Math.Acos(abNorm.X * bcNorm.X + abNorm.Y * bcNorm.Y + abNorm.Z * bcNorm.Z);

            // Return the angle in degrees
            return rad * 180 / Math.PI;
        }

        // Calculates the frontal 2D angle produced by three points a, b and c
        // Each of them must be 3D, though z is not used (just x and y are needed for frontal angles)
        // The angle which is calculated is BAC, thus the second point is always used as vertex
        // Returns null if any of the three points have missing data
        public static double? Calculate2DFrontalAngle(Point3D? a, Point3D? b, Point3D? c)
        {
            if (a == null || b == null || c == null)
                return null;

            // Calculate the absolute angles for points a and c
            double AbsoluteAngleA = Math.Atan2(a.Value.Y - b.Value.Y, a.Value.X - b.Value.X);
            double AbsoluteAngleC = Math.Atan2(c.Value.Y - b.Value.Y, c.Value.X - b.Value.X);

            // Calculate the radiant angle by making the difference between the absolute ones and then convert it to normal degrees
            return (AbsoluteAngleA - AbsoluteAngleC) * 180 / Math.PI;
        }

        // Calculates the lateral 2D angle produced by three points a, b and c
        // Each of them must be 3D, though x is not used (just y and z are needed for lateral angles)
        // The angle which is calculated is BAC, thus the second point is always used as vertex
        // Returns null if any of the three points have missing data
        public static double? Calculate2DLateralAngle(Point3D? a, Point3D? b, Point3D? c)
        {
            if (a == null || b == null || c == null)
                return null;

            // Calculate the absolute angles for points a and c
            double AbsoluteAngleA = Math.Atan2(a.Value.Y - b.Value.Y, a.Value.Z - b.Value.Z);
            double AbsoluteAngleC = Math.Atan2(c.Value.Y - b.Value.Y, c.Value.Z - b.Value.Z);

            // Calculate the radiant angle by making the difference between the absolute ones and then convert it to normal degrees
            return (AbsoluteAngleA - AbsoluteAngleC) * 180 / Math.PI;
        }

        // Calculates the overhead 2D angle produced by three points a, b and c
        // Each of them must be 3D, though y is not used (just x and z are needed for overhead angles)
        // The angle which is calculated is BAC, thus the second point is always used as vertex
        // Returns null if any of the three points have missing data
        public static double? Calculate2DOverheadAngle(Point3D? a, Point3D? b, Point3D? c)
        {
            if (a == null || b == null || c == null)
                return null;

            // Calculate the absolute angles for points a and c
            double AbsoluteAngleA = Math.Atan2(a.Value.X - b.Value.X, a.Value.Z - b.Value.Z);
            double AbsoluteAngleC = Math.Atan2(c.Value.X - b.Value.X, c.Value.Z - b.Value.Z);

            // Calculate the radiant angle by making the difference between the absolute ones and then convert it to normal degrees
            return (AbsoluteAngleA - AbsoluteAngleC) * 180 / Math.PI;
        }
    }
}
