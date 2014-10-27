using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class KneelingPosition
    {
        private double height;
        private double kneelingHeight;
        public KneelingPosition(Skeleton ske){
            Joint kneeRight = ske.Joints[JointType.KneeRight];
            Joint kneeLeft = ske.Joints[JointType.KneeLeft];
            Joint ankleRight = ske.Joints[JointType.AnkleRight];
            Joint ankleLeft = ske.Joints[JointType.AnkleLeft];
            Joint head = ske.Joints[JointType.Head];
            double kry = kneeRight.Position.Y;
            //double krx = kneeRight.Position.X;
            double kly = kneeLeft.Position.Y;
            //double klx = kneeLeft.Position.X;
            double ary = ankleRight.Position.Y;
            //double arx = ankleRight.Position.X;
            double aly = ankleLeft.Position.Y;
            //double alx = ankleLeft.Position.X;
            double hy = head.Position.Y;
            height = hy - ary;
            //double kneeRightToAnkleRight = System.Math.Sqrt(((krx - arx) * (krx-arx)) + ((kry-ary) * (kry-ary)));
            //double kneeLeftToAnkleLeft = System.Math.Sqrt(((klx-alx) * (klx-alx)) + ((kly-aly) * (kly-aly)));
            double kneeRightToAnkleRight = kry - ary;
            double kneeLeftToAnkleLeft = kly - aly;
            kneelingHeight = height - ((kneeRightToAnkleRight + kneeLeftToAnkleLeft) / 2);
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
            Joint head = ske.Joints[JointType.Head];
            double hy = head.Position.Y;
            double hcy = hipCenter.Position.Y;
            double hcx = hipCenter.Position.X;
            double kry = kneeRight.Position.Y;
            double krz = kneeRight.Position.Z;
            double krx = kneeRight.Position.X;
            double kly = kneeLeft.Position.Y;
            double klz = kneeLeft.Position.Z;
            double klx = kneeLeft.Position.X;
            double arz = ankleRight.Position.Z;
            double alz = ankleLeft.Position.Z;
            double ary = ankleRight.Position.Y;
            double aly = ankleLeft.Position.Y;
            double hry = hipRight.Position.Y;
            double hrz = hipRight.Position.Z;
            double hly = hipLeft.Position.Y;
            double hlz = hipLeft.Position.Z;
            double frz = footRight.Position.Z;
            double flz = footLeft.Position.Z;

            double hipCenterToKneeRight = System.Math.Sqrt(((hcx - krx) * (hcx - krx)) + ((hcy - kry) * (hcy - kry)));
            double hipCenterToKneeLeft = System.Math.Sqrt(((hcx - klx) * (hcx - klx)) + ((hcy - kly) * (hcy - kly)));
            double kneeRightToKneeLeft = System.Math.Sqrt(((krx - klx) * (krx - klx)) + ((kry - kly) * (kry - kly)));
            double angleCalculated = System.Math.Acos(((hipCenterToKneeRight * hipCenterToKneeRight) + (hipCenterToKneeLeft * hipCenterToKneeLeft) - (kneeRightToKneeLeft * kneeRightToKneeLeft)) / (2 * hipCenterToKneeRight * hipCenterToKneeLeft));
            angleCalculated = angleCalculated * 180 / System.Math.PI;

            double hipRightToKneeRight = System.Math.Sqrt(((hrz - krz) * (hrz - krz)) + ((hry - kry) * (hry - kry)));
            double hipLeftToKneeLeft = System.Math.Sqrt(((hlz - klz) * (hlz - klz)) + ((hly - kly) * (hly - kly)));
            
            double kneeRightToAnkleRight = kry - ary;
            double kneeLeftToAnkleLeft = kly - aly;
            double kneeToAnkleCalculated = (kneeRightToAnkleRight + kneeLeftToAnkleLeft) / 2;
            double error = System.Math.Abs(5 * kneelingHeight / 100);
            double heightCalculated = hy - ary - kneeToAnkleCalculated;

            if (((krz < arz) || (ankleRight.TrackingState == JointTrackingState.Inferred)) && ((klz < alz) || (ankleLeft.TrackingState == JointTrackingState.Inferred)) && (footLeft.TrackingState == JointTrackingState.Inferred) && (footRight.TrackingState == JointTrackingState.Inferred) && (angle + 5 > angleCalculated) && (angle - 5 < angleCalculated) && (heightCalculated < kneelingHeight + error) && (heightCalculated > kneelingHeight - error))
                return true;

            return false;
        }
    }
}
