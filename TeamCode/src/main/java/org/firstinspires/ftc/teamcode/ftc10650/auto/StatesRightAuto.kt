//package org.firstinspires.ftc.teamcode.ftc10650.auto
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.Disabled
//import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData
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
//@Autonomous(name = "Right Auto States", group = "ftc10650")
//class StatesRightAuto : ComplexOp() {
//
//    override fun startPositionAndOrientation(): StartData {
//        return StartData(Vector2D(0.0, 0.0), 0.0)
//    }
//
//    @Throws(InterruptedException::class)
//    override fun body() {
//        RobotMap.gyro.resetYaw()
//
//        val signalPosition = d.robot.aprilTagDetectionPipeline.id
//
//
//        RobotMap.leftCamera.setPipeline(d.robot.leftPoleAlignPipeline)
//
//
//        ComplexMove(
//                SpeedCalcs.StandardRampUpDown(0.1, 0.35, 0.5),
//                MotionCalcs.pointSplineMotion(0.95,
//                    Vector2D(-.2,0.1),
//                    Vector2D(-.2, 2.10),
//                    Vector2D(-0.6, 2.10),
//                ),
//                OrientationCalcs.lookToOrientation(0.0)
//        )
//        ComplexMove(null,null,OrientationCalcs.lookToOrientation(0.0), OtherCalcs.Raise(), OtherCalcs.AutoAlignPost())
//
//
//        for (i in 0.. 1){
//
//            ComplexMove(
//                SpeedCalcs.StandardRampUpDown(0.15, .25, .5),
//                MotionCalcs.pointSplineMotion(.95,
//                    Vector2D(0.5,2.2),
//                    Vector2D(0.795,2.11),//.795, 1.96
//                ),
//                OrientationCalcs.spinToProgress(OrientationCalcs.spinProgress(0.1, 0.5, -90.0), ),
//                OtherCalcs.LiftToPos(270 - 80 * i),
//                OtherCalcs.OtherLambda { d: MoveData ->
//                    if (d.progress > .5) {
//                        RobotMap.pitch.position = .46
//                    } else {
//                        RobotMap.pitch.position = .75
//                    } },
//            )
//            ComplexMove(null, null, null,
//                OtherCalcs.TimeProgress(600.0),
//
//                OtherCalcs.OtherLambda { RobotMap.claw.position = .23 })
//
//            ComplexMove(
//                SpeedCalcs.SetSpeed(.1),
//                MotionCalcs.MotionLambda { Vector2D(-1.0, 0.0) },
//                OrientationCalcs.lookToOrientation(-90.0),
//                OtherCalcs.TimeProgress(300.0),
//                OtherCalcs.OtherLambda {
//                    if(it.progress>.2) {
//                        RobotMap.lift.targetPosition = 550 - 80*i
//                        RobotMap.liftEx.velocity = 1500.0
//                        RobotMap.pitch.position = .55
//                    }
//                }
//
//            )
//
//            ComplexMove(
//                SpeedCalcs.StandardRampUpDown(0.15, 0.25, 0.5),
//                MotionCalcs.pointSplineMotion(0.95,
//                    Vector2D(0.75, 2.2),
//                    Vector2D(-0.6, 2.1),
//                ),
//                OrientationCalcs.spinToProgress(OrientationCalcs.spinProgress(0.5, 0.9, 0.0)),
//                OtherCalcs.OtherLambda { RobotMap.pitch.position = 0.9 }
//            )
//            ComplexMove(null,null,OrientationCalcs.lookToOrientation(0.0), OtherCalcs.Raise(), OtherCalcs.AutoAlignPost())
//
//            //fudge
//            d.wPos = d.wPos + Vector2D(-0.08, 0.0)
//        }
//
//
//
//
//        if(signalPosition == 0) {
//            ComplexMove(
//                    SpeedCalcs.SetSpeed(.2),
//                    MotionCalcs.pointSplineMotion(
//                            0.95,
//                            Vector2D(-1.2, 2.25),
//                            Vector2D(-1.2, 2.0),
//                    ),
//                    OrientationCalcs.lookToOrientation(0.0),
//                    OtherCalcs.LiftToPos(5),
//                    OtherCalcs.OtherLambda { RobotMap.pitch.position = .8
//                    RobotMap.claw.position = .3}
//            )
//        }
//        else if (abs(signalPosition) == 1) {
//            ComplexMove(
//                    SpeedCalcs.SetSpeed(.2),
//                    MotionCalcs.pointSplineMotion(
//                            0.95,
//                            Vector2D(-0.3, 2.0),
//                    ),
//                    OrientationCalcs.lookToOrientation(0.0),
//                OtherCalcs.LiftToPos(5),
//                OtherCalcs.OtherLambda { RobotMap.pitch.position = .8
//                    RobotMap.claw.position = .3}
//            )
//        } else if (signalPosition == 2){
//            ComplexMove(
//                    SpeedCalcs.SetSpeed(.2),
//                    MotionCalcs.pointSplineMotion(.95,
//                            Vector2D(0.7, 2.25),
//                    ),
//                    OrientationCalcs.lookToOrientation(0.0),
//                OtherCalcs.LiftToPos(5),
//                OtherCalcs.OtherLambda { RobotMap.pitch.position = .8
//                    RobotMap.claw.position = .3})
//        }
//
//    }
//
//}