package org.firstinspires.ftc.teamcode.ftc10650.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.calculators.*
import org.firstinspires.ftc.teamcode.op.ComplexOp
import org.firstinspires.ftc.teamcode.utilities.Vector2D
import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap
import kotlin.math.abs
@Disabled
@Autonomous(name = "Left Auto States", group = "ftc10650")
class StatesLeftAuto: ComplexOp(){

    override fun startPositionAndOrientation(): Interfaces.MoveData.StartData {
        return Interfaces.MoveData.StartData(Vector2D(0.0, 0.0), 0.0)
    }

    @Throws(InterruptedException::class)
    override fun body() {

        RobotMap.gyro.resetYaw()

        val signalPosition = d.robot.aprilTagDetectionPipeline.id

        d.debugData1 = signalPosition.toDouble();

        RobotMap.camera.setPipeline(d.robot.leftPoleAlignPipeline)


        ComplexMove(
            SpeedCalcs.StandardRampUpDown(0.1, 0.35, 0.5),
            MotionCalcs.pointSplineMotion(0.95,
                Vector2D(-.2,.1),
                Vector2D(-.2, 2.1),
                Vector2D(0.45, 2.1),
            ),
            OrientationCalcs.lookToOrientation(0.0)
        )

        ComplexMove(null,null,OrientationCalcs.lookToOrientation(0.0), OtherCalcs.Raise(), OtherCalcs.AutoAlignPost())


        for (i in 0.. 1){

            ComplexMove(
                SpeedCalcs.StandardRampUpDown(0.15, .25, .5),
                MotionCalcs.pointSplineMotion(.95,
                    Vector2D(-0.8,2.2),
                    Vector2D(-1.05,2.25)//-1.05, 2.1
                ),
                OrientationCalcs.spinToProgress(OrientationCalcs.spinProgress(0.1, 0.5, 90.0), ),
                OtherCalcs.LiftToPos(270 - 80 * i),
                OtherCalcs.OtherLambda { d: Interfaces.MoveData ->
                    if (d.progress > .5) {
                        RobotMap.pitch.position = .46
                    } else {
                        RobotMap.pitch.position = .75
                    } },
            )
            ComplexMove(null, null, null,
                OtherCalcs.TimeProgress(600.0),

                OtherCalcs.OtherLambda { RobotMap.claw.position = .23 })

            ComplexMove(
                SpeedCalcs.SetSpeed(.1),
                MotionCalcs.MotionLambda { Vector2D(1.0, 0.0) },
                OrientationCalcs.lookToOrientation(90.0),
                OtherCalcs.TimeProgress(300.0),
                OtherCalcs.OtherLambda {
                    if(it.progress>.2) {
                        RobotMap.lift.targetPosition = 550 - 80*i
                        RobotMap.liftEx.velocity = 1500.0
                        RobotMap.pitch.position = .55
                    }
                }

            )

            ComplexMove(
                SpeedCalcs.StandardRampUpDown(0.15, 0.25, 0.5),
                MotionCalcs.pointSplineMotion(0.95,
                    Vector2D(-0.95, 2.10),
                    Vector2D(0.45, 2.10)
                ),
                OrientationCalcs.spinToProgress(OrientationCalcs.spinProgress(0.5, 0.9, 0.0)),
                OtherCalcs.OtherLambda { RobotMap.pitch.position = 0.9 }
            )
            ComplexMove(null,null,OrientationCalcs.lookToOrientation(0.0), OtherCalcs.Raise(), OtherCalcs.AutoAlignPost())

            //fudge
            d.wPos = d.wPos + Vector2D(0.08, -0.05)
        }

        if(signalPosition == 0) {
            ComplexMove(
                SpeedCalcs.StandardRampUpDown(.2, .35, .5),
                MotionCalcs.pointSplineMotion(
                    0.95,
                    Vector2D(-0.95, 2.1),
                ),
                OrientationCalcs.lookToOrientation(0.0),
                OtherCalcs.LiftToPos(5),
                OtherCalcs.OtherLambda { RobotMap.pitch.position = .8
                    RobotMap.claw.position = .3}
            )
        }
        else if (abs(signalPosition) == 1) {
            ComplexMove(
                SpeedCalcs.SetSpeed(.2),
                MotionCalcs.pointSplineMotion(0.95,
                    Vector2D(0.0, 2.1),
                ),
                OrientationCalcs.lookToOrientation(0.0),
                OtherCalcs.LiftToPos(5),
                OtherCalcs.OtherLambda { RobotMap.pitch.position = .8
                    RobotMap.claw.position = .3}
            )
        } else if (signalPosition == 2){
            ComplexMove(
                SpeedCalcs.SetSpeed(.2),
                MotionCalcs.pointSplineMotion(0.95,
                    Vector2D(0.9, 2.1),
                ),
                OrientationCalcs.lookToOrientation(0.0),
                OtherCalcs.LiftToPos(5),
                OtherCalcs.OtherLambda { RobotMap.pitch.position = .8
                    RobotMap.claw.position = .3}
            )
        }

    }
}