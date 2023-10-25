package org.firstinspires.ftc.teamcode.ftc10650.auto;//package org.firstinspires.ftc.teamcode.ftc10650.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Calculators.*;
//import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData;
//import org.firstinspires.ftc.teamcode.Op.ComplexOp;
//import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
//
//@Autonomous(name = "DonutAuto", group = "ftc10650")
//public class AutoAuto extends ComplexOp {
//
//    @Override
//    public MoveData.StartData startPositionAndOrientation() {
//        /**
//         * @see StartPositionAndOrientation is used at the beginning of every game code
//         * this includes tele-op and autonomous
//         */
//        return new MoveData.StartData(new Vector2D(0,0), 0);
//    }
//
//    @Override
//    public void body() throws InterruptedException {
////        ComplexMove(null,null,null,OtherCalcs.Shoot());
////        ComplexMove(null, null, null, OtherCalcs.GetDonutStack());
//
//
////        ComplexMove(
////                null,
////                null,
////                OrientationCalcs.lookToPower(),
////                OtherCalcs.SingleShot(5000),
////                OtherCalcs.GetPowerPositions()
////        );
////        ComplexMove(
////                null,
////                null,
////                OrientationCalcs.lookToPower(),
////                OtherCalcs.SingleShot(5000),
////                OtherCalcs.GetPowerPositions()
////        );
//        if(d.stackHeight == 0){
//
//
//            ComplexMove(//dropping first wobble
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.3, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.7, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.3,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
//                    ),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(12, 85)),
//                    //OrientationCalcs.lookToOrientation(0));
//                    OrientationCalcs.spinToProgress(
//                            new OrientationCalcs.spinProgress(0.0, 0.9, 0)));
//
//
//            ComplexMove(null, null, null,
//                    OtherCalcs.SetWobblePosition(550),//60
//                    OtherCalcs.TimeProgress(1000));
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.7, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2, 1.0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
//                    ),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(-13, 26)),
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.SetGrabberPosition(true)
//            );
//
//            ComplexMove(null, null, null,
//                    OtherCalcs.SetGrabberPosition(false),
//                    OtherCalcs.TimeProgress(1000));
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.7, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
//                    ),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(12, 77)),
//                    //OrientationCalcs.lookToOrientation(0));
//                    OrientationCalcs.spinToProgress(
//                            new OrientationCalcs.spinProgress(0.0, 0.9, 90)));
//
////            ComplexMove(null, null, null,
////                    OtherCalcs.SetGrabberPosition(true),
////                    OtherCalcs.TimeProgress(1000));
//
//            ComplexMove(null, null, null,
//                    OtherCalcs.SetGrabberPosition(true),
//                    OtherCalcs.TimeProgress(1000));
//
//            ComplexMove(null, null, null, OtherCalcs.SetBucketPositionWithProgress(0.24));
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5, new Vector2D(-25, 55)),
//                    OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.2, 0.9, 0.0))
//            );
//            ComplexMove(
//                    null,
//                    null,
//                    OrientationCalcs.lookToPower(),
//                    OtherCalcs.SingleShot(1500, 2000),
//                    OtherCalcs.GetPowerPositions()
//            );
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.8, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
//                    ),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(-13, 70)),
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.SetWobblePosition(3));
//
//
//            ComplexMove(null, null,
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.TimeProgress(5000));
//
//
//        } else if (d.stackHeight == 1) {
//
//            ComplexMove(null, null, null, OtherCalcs.Shoot(), OtherCalcs.Intake(true));
//
//            ComplexMove(//dropping first wobble
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
//                    ),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(-12, 105)),
//                    //OrientationCalcs.lookToOrientation(0));
//                    OrientationCalcs.spinToProgress(
//                            new OrientationCalcs.spinProgress(0.0, 0.9, 0)));
//
//
//            ComplexMove(null, null, null,
//                    OtherCalcs.SetWobblePosition(550),//60
//                    OtherCalcs.TimeProgress(1000));
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.6,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2, 1.0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
//                    ),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(-20, 95),
//                            new Vector2D(-13, 17)),
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.SetGrabberPosition(true)
//            );
//
//            ComplexMove(null, null, null,
//                    OtherCalcs.SetGrabberPosition(false),
//                    OtherCalcs.TimeProgress(1000));
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
//                    ),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(-12, 100)),
//                    //OrientationCalcs.lookToOrientation(0));
//                    OrientationCalcs.spinToProgress(
//                            new OrientationCalcs.spinProgress(0.0, 0.9, 90)));
//
//
////            ComplexMove(
////                    SpeedCalcs.SetProgressSpeed(
////                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                            new SpeedCalcs.ProgressSpeed(1.0, 0.6, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
////                    MotionCalcs.PointMotion(5,
//////                            new Vector2D(-30, 30),
//////                            new Vector2D(-30, 55),
////                            new Vector2D(-20, 85)),
////                    OrientationCalcs.spinToProgress(
////                            new OrientationCalcs.spinProgress(0, 0.9, 0)));
////
////
////            ComplexMove(null, null, null,
////                    OtherCalcs.SetWobblePosition(550),//60
////                    OtherCalcs.TimeProgress(1000));
////
////
////            ComplexMove(
////                    SpeedCalcs.SetSpeed(0.3),
////                    MotionCalcs.PointMotion(5,
////                            new Vector2D(-15, 70)),
////                    OrientationCalcs.holdHeading(),
////                    OtherCalcs.SetWobblePosition(3));
////
////
////            ComplexMove(null, null,
////                    OrientationCalcs.spinToProgress(
////                            new OrientationCalcs.spinProgress(0.0, 0.9, 0)),
////                    OtherCalcs.TimeProgress(5000));
//
//
//            ComplexMove(null, null, null,
//                    OtherCalcs.SetGrabberPosition(true),
//                    OtherCalcs.TimeProgress(1000));
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5, new Vector2D(-30, 55)),
//                    OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.2, 0.9, 0.0))
//            );
//            ComplexMove(
//                    null,
//                    null,
//                    OrientationCalcs.lookToPower(),
//                    OtherCalcs.SingleShot(1500, 2000),
//                    OtherCalcs.GetPowerPositions()
//            );
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
//                    ),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(-15, 70)),
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.SetWobblePosition(3));
//
//
//            ComplexMove(null, null,
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.TimeProgress(5000));
//
//        } else if (d.stackHeight == 4) {
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0.0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.8, 0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.7, 0.7, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2, 1.0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(0,
//                            new Vector2D(0, 20),
//                            new Vector2D(-10, 30),
//                            new Vector2D(-10, 35)),
//                    OrientationCalcs.lookToOrientation(3),
//                    OtherCalcs.SetBucketPosition(0.24),
//                    OtherCalcs.SetShooterSpeed(1690)
//            );
//
//            ComplexMove(
//                    null, null,
//                    OrientationCalcs.lookToOrientation(-3),
//                    OtherCalcs.Shooting(3));
//            ComplexMove(null, null, null, OtherCalcs.SetBucketPosition(0.68),
//                    OtherCalcs.TimeProgress(700));
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.02, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.03, 0.03, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.7, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5,
////                            new Vector2D(-30, 30),
////                            new Vector2D(-30, 55),
////                            new Vector2D(0,27),
//                            new Vector2D(-5, 44),
//                            new Vector2D(14, 44),
//                            new Vector2D(14, 117)),
//                    OrientationCalcs.spinToProgress(
//                            new OrientationCalcs.spinProgress(0, 0.3, 0),
//                            new OrientationCalcs.spinProgress(0.4, 0.8, 80)),
//                    OtherCalcs.Intake(true),
//                    OtherCalcs.SetBucketPosition(0.68));
//            ComplexMove(null, null, null,
////                    OrientationCalcs.spinToProgress(
////                            new OrientationCalcs.spinProgress(0, 0.3, 0),
////                          new OrientationCalcs.spinProgress(0.4, 0.8, 45)),
//                    OtherCalcs.SetWobblePosition(550),//60
//                    OtherCalcs.TimeProgress(1000));
//
////            ComplexMove(
////                    SpeedCalcs.SetProgressSpeed(
////                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                            new SpeedCalcs.ProgressSpeed(1.0, 0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                            new SpeedCalcs.ProgressSpeed(1.0,0.7, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
////                    MotionCalcs.PointMotion(5,
//////                            new Vector2D(-30, 30),
//////                            new Vector2D(-30, 55),
////                            new Vector2D(0,27),
////                            new Vector2D(16, 115)),
////                    OrientationCalcs.spinToProgress(
//////                            new OrientationCalcs.spinProgress(0, 0.3, 0),
////                            new OrientationCalcs.spinProgress(0.4, 0.8, 45)));//45
//
////            ComplexMove(null, null, null,
////                    OtherCalcs.SetWobblePosition(550),//60
////                    OtherCalcs.TimeProgress(1000));
//
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0, 0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.7, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2, 1.0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
//                    ),
//                    MotionCalcs.PointMotion(5,
////                            new Vector2D(15,45),
//                            new Vector2D(-8, 23)),
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.SetGrabberPosition(true)
//            );
//
//            ComplexMove(null, null, null,
//                    OtherCalcs.SetGrabberPosition(false),
//                    OtherCalcs.TimeProgress(500));
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0, 0.2, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.8, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2, 1.0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5,
////                            new Vector2D(12,45),
//                            new Vector2D(14, 112)),
//                    OrientationCalcs.spinToProgress(
////                            new OrientationCalcs.spinProgress(0, 0.3, 0),
//                            new OrientationCalcs.spinProgress(0.3, 0.9, 135)));
//
//            ComplexMove(null, null, null,
//                    OtherCalcs.SetGrabberPosition(true),
//                    OtherCalcs.Intake(-1000),
//                    OtherCalcs.TimeProgress(500));
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5, new Vector2D(-23, 55)),
//                    OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.2, 0.9, 0.0)),
//                    OtherCalcs.Intake(false)
//            );
//            ComplexMove(
//                    null,
//                    null,
//                    OrientationCalcs.lookToPower(),
//                    OtherCalcs.SingleShot(1000, 0),
//                    OtherCalcs.GetPowerPositions()
//            );
//
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.2, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(1.0,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
//                    ),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(-15, 70)),
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.SetWobblePosition(3));
//
//
//            ComplexMove(null, null,
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.TimeProgress(5000));
//
//        }
//
//
//    }
//}
