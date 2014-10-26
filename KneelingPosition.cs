using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class KneelingPosition
    {

        public KneelingPosition(){

        }

        public bool correctPosition(Skeleton ske, double angle)
        {
            Joint kneeRight = ske.Joints[JointType.KneeRight];
            Joint kneeLeft = ske.Joints[JointType.KneeLeft];
            Joint ankleRight = ske.Joints[JointType.AnkleRight];
            Joint ankleLeft = ske.Joints[JointType.AnkleLeft];
            Joint hipCenter = ske.Joints[JointType.HipCenter];
            Joint hipRight = ske.Joints[JointType.HipRight];
            Joint hipLeft = ske.Joints[JointType.HipLeft];
            Joint footLeft = ske.Joints[JointType.FootLeft];
            Joint footRight = ske.Joints[JointType.FootRight];
            double hcy = hipCenter.Position.Y;
            double hcx = hipCenter.Position.X;
            double kry = kneeRight.Position.Y;
            double krz = kneeRight.Position.Z;
            double krx = kneeRight.Position.X;
            double kly = kneeLeft.Position.Y;
            double klz = kneeLeft.Position.Z;
            double klx = kneeLeft.Position.X;

            double ary = ankleRight.Position.Y;
            double arz = ankleRight.Position.Z;
            double aly = ankleLeft.Position.Y;
            double alz = ankleLeft.Position.Z;
            double hry = hipRight.Position.Y;
            double hrz = hipRight.Position.Z;
            double hly = hipLeft.Position.Y;
            double hlz = hipLeft.Position.Z;

            double hipCenterToKneeRight = System.Math.Sqrt(((hcx - krx) * (hcx - krx)) + ((hcy - kry) * (hcy - kry)));
            double hipCenterToKneeLeft = System.Math.Sqrt(((hcx - klx) * (hcx - klx)) + ((hcy - kly) * (hcy - kly)));
            double kneeRightToKneeLeft = System.Math.Sqrt(((krx - klx) * (krx - klx)) + ((kry - kly) * (kry - kly)));
            double angleCalculated = System.Math.Acos(((hipCenterToKneeRight * hipCenterToKneeRight) + (hipCenterToKneeLeft * hipCenterToKneeLeft) - (kneeRightToKneeLeft * kneeRightToKneeLeft)) / (2 * hipCenterToKneeRight * hipCenterToKneeLeft));
            angleCalculated = angleCalculated * 180 / System.Math.PI;

            double hipRightToKneeRight = System.Math.Sqrt(((hrz - krz) * (hrz - krz)) + ((hry - kry) * (hry - kry)));
            double hipLeftToKneeLeft = System.Math.Sqrt(((hlz - klz) * (hlz - klz)) + ((hly - kly) * (hly - kly)));
            double kneeRightToAnkleRight = System.Math.Sqrt(((krz - arz) * (krz - arz)) + ((kry - ary) * (kry - ary)));
            double kneeLeftToAnkleLeft = System.Math.Sqrt(((klz - alz) * (klz - alz)) + ((kly - aly) * (kly - aly)));

            if ((kneeLeft.TrackingState == JointTrackingState.Tracked) && (kneeRight.TrackingState == JointTrackingState.Tracked) && hipRightToKneeRight > kneeRightToAnkleRight && hipLeftToKneeLeft > kneeLeftToAnkleLeft && (krz < arz) && (ankleRight.TrackingState == JointTrackingState.Inferred) && (klz < alz) && (ankleLeft.TrackingState == JointTrackingState.Inferred) && (footLeft.TrackingState == JointTrackingState.Inferred) && (footRight.TrackingState == JointTrackingState.Inferred) && angle + 5 > angleCalculated && angle - 5 < angleCalculated)
                return true;

            return false;
        }
    }
}
