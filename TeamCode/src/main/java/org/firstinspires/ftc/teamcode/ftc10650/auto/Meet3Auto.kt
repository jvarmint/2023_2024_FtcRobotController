//package org.firstinspires.ftc.teamcode.ftc10650.auto
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.Disabled
//import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData.StartData
//import org.firstinspires.ftc.teamcode.calculators.MotionCalcs
//import org.firstinspires.ftc.teamcode.calculators.OrientationCalcs
//import org.firstinspires.ftc.teamcode.calculators.OtherCalcs
//import org.firstinspires.ftc.teamcode.calculators.SpeedCalcs
//import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap
//import org.firstinspires.ftc.teamcode.op.ComplexOp
//import org.firstinspires.ftc.teamcode.utilities.Vector2D
//
////import kotlin.Math.sign
//@Disabled
//@Autonomous(name = "meet 3 auto Left", group = "ftc10650")
//class Meet3Auto : ComplexOp() {
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
//        var signalPosition = d.robot.signalPipeline.currentSignal
//        while (signalPosition == -1){
//            signalPosition = d.robot.signalPipeline.currentSignal
//        }
//
//        ComplexMove(
//                SpeedCalcs.SetSpeed(.2),
//                MotionCalcs.pointSplineMotion(0.95,
//                        Vector2D(-.1,.1),
//                        Vector2D(-.1, 1.90),
//                        Vector2D(0.4, 2.1)
//                ),
//                OrientationCalcs.lookToOrientation(0.0)
//        )
//        ComplexMove(null,MotionCalcs.AlignPost(),OrientationCalcs.lookToOrientation(0.0), OtherCalcs.Raise())
//        ComplexMove(null,null,OrientationCalcs.lookToOrientation(0.0),OtherCalcs.TimeProgress(1000.0))
//        ComplexMove(null,null,OrientationCalcs.lookToOrientation(0.0), OtherCalcs.Lower())
//        ComplexMove(null,null,OrientationCalcs.lookToOrientation(0.0),OtherCalcs.TimeProgress(1000.0))
//
//
//        if(signalPosition == 1) { // purple
//            ComplexMove(
//                    SpeedCalcs.SetSpeed(.2),
//
//                    MotionCalcs.pointSplineMotion(
//                            0.95,
//                            Vector2D(-0.9, 2.1),
//                            Vector2D(-0.9, 1.2),
//                    ),
//                    OrientationCalcs.lookToOrientation(0.0)
//            )
//        }
//        else if (signalPosition == 2) { // green
//            ComplexMove(
//                    SpeedCalcs.SetSpeed(.2),
//
//                    MotionCalcs.pointSplineMotion(
//                            0.95,
//                            Vector2D(-0.1, 2.1),
//                            Vector2D(-0.1, 1.2),
//                    ),
//                    OrientationCalcs.lookToOrientation(0.0)
//            )
//        } else if (signalPosition == 3){ // orange
//            ComplexMove(
//                    SpeedCalcs.SetSpeed(.2),
//                    MotionCalcs.pointSplineMotion(0.95,
//                            Vector2D(0.75, 2.1),
//                            Vector2D(0.75, 1.2)
//                    ),
//                    OrientationCalcs.lookToOrientation(0.0)
//            )
//        }
//
//    }
//
//}