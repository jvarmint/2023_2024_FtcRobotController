package org.firstinspires.ftc.teamcode.ftc10650.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData.StartData
import org.firstinspires.ftc.teamcode.calculators.MotionCalcs
import org.firstinspires.ftc.teamcode.calculators.OrientationCalcs
import org.firstinspires.ftc.teamcode.calculators.SpeedCalcs
import org.firstinspires.ftc.teamcode.op.ComplexOp
import org.firstinspires.ftc.teamcode.utilities.Vector2D
import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap

//import kotlin.Math.sign
@Disabled
@Autonomous(name = "TESTS", group = "ftc10650")
class StatesTESTv0 : ComplexOp() {

    override fun startPositionAndOrientation(): StartData {
        return StartData(Vector2D(0.0, 0.0), 0.0)
    }

    @Throws(InterruptedException::class)
    override fun body() {
        RobotMap.gyro.resetYaw()
        ComplexMove(
            SpeedCalcs.StandardRampUpDown(.1, .1, .2),
//                MotionCalcs.AlignPost(),
            MotionCalcs.pointSplineMotion(0.8,
                Vector2D(0.0, 2.0),
                Vector2D(-2.0,2.0),
                Vector2D(-2.0,0.0),
                Vector2D(0.0,0.0),
            ),
            OrientationCalcs.spinToProgress(OrientationCalcs.spinProgress(.1, .9, 0.0))
        )

//        ComplexMove(
//            SpeedCalcs.StandardRampUpDown(.1, .5, .2),
////                MotionCalcs.AlignPost(),
//            MotionCalcs.pointSplineMotion(0.9,
//                Vector2D(2.0, 0.0),
//                Vector2D(2.0, 1.0),
//                Vector2D(0.0, 1.0),
//                Vector2D(0.0, 0.0)
//            ),
//            OrientationCalcs.lookToOrientation(0.0)
//        )

    }

}