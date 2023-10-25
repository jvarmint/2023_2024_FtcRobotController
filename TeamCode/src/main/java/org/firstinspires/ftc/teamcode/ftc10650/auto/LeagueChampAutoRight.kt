//package org.firstinspires.ftc.teamcode.ftc10650.auto
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.Disabled
//import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData.StartData
//import org.firstinspires.ftc.teamcode.calculators.MotionCalcs
//import org.firstinspires.ftc.teamcode.calculators.OrientationCalcs
//import org.firstinspires.ftc.teamcode.calculators.OtherCalcs
//import org.firstinspires.ftc.teamcode.calculators.SpeedCalcs
//import org.firstinspires.ftc.teamcode.op.ComplexOp
//import org.firstinspires.ftc.teamcode.utilities.Vector2D
//import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap
//import kotlin.math.abs
//
////import kotlin.Math.sign
//@Disabled
//@Autonomous(name = "auto right", group = "ftc10650")
//class LeagueChampAutoRight : ComplexOp() {
//
//    override fun startPositionAndOrientation(): StartData {
//        return StartData(Vector2D(0.0, 0.0), 0.0)
//    }
//
//    @Throws(InterruptedException::class)
//    override fun body() {
//        RobotMap.gyro.resetYaw()
//
//        ComplexMove(null, null,null,OtherCalcs.ShakeClaw(), OtherCalcs.TimeProgress(2000.0));
//
////        var signalPosition = d.robot.aprilTagDetectionPipeline.id
////        var count = 0
////        while (signalPosition == -1 && count < 500){
////            signalPosition = d.robot.aprilTagDetectionPipeline.id
////            count++
////        }
//        val signalPosition = d.robot.aprilTagDetectionPipeline.id;
//
//        RobotMap.leftCamera.setPipeline(d.robot.leftPoleAlignPipeline)
//
////        saveInitialHeading(true)
//
//        ComplexMove(
//                SpeedCalcs.SetSpeed(.2),
////                MotionCalcs.AlignPost(),
//                MotionCalcs.pointSplineMotion(0.95,
//                    Vector2D(-.1,0.1),
//                    Vector2D(-.15, 2.02),
//                    Vector2D(-0.75, 2.04)//was -0.5, 1.95
//                ),
//                OrientationCalcs.lookToOrientation(0.0)
//        )
//        ComplexMove(null,null,null, OtherCalcs.Raise(), OtherCalcs.AutoAlignPost())
//        ComplexMove(null,null,null,OtherCalcs.TimeProgress(1000.0), OtherCalcs.AutoAlignPost())
//        ComplexMove(null,null,null, OtherCalcs.Lower())
//        ComplexMove(null,null,null,OtherCalcs.TimeProgress(1000.0))
//
//
//        if(signalPosition == 0) { // purple
//            ComplexMove(
//                    SpeedCalcs.SetSpeed(.2),
////                MotionCalcs.AlignPost(),
//                    MotionCalcs.pointSplineMotion(
//                            0.0,
//                            Vector2D(-1.1, 2.0),
//                            Vector2D(-1.1, 1.2),
//                    ),
//                    OrientationCalcs.lookToOrientation(0.0)
//            )
//        }
//        else if (abs(signalPosition) == 1 /* || signalPosition == -1*/) { // green
//            ComplexMove(
//                    SpeedCalcs.SetSpeed(.2),
////                MotionCalcs.AlignPost(),
//                    MotionCalcs.pointSplineMotion(
//                            0.0,
//                            Vector2D(-0.1, 2.0),
//                            Vector2D(-0.1, 1.2),
//                    ),
//                    OrientationCalcs.lookToOrientation(0.0)
//            )
//        } else if (signalPosition == 2){ // orange
//            ComplexMove(
//                    SpeedCalcs.SetSpeed(.2),
////                MotionCalcs.AlignPost(),
//                    MotionCalcs.pointSplineMotion(.95,
//                            Vector2D(0.65, 2.0),
//                            Vector2D(0.65, 1.2)
//                    ),
//                    OrientationCalcs.lookToOrientation(0.0))
//        }
//
//    }
//
//}