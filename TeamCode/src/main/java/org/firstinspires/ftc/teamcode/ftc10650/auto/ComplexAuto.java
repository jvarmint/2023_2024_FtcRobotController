package org.firstinspires.ftc.teamcode.ftc10650.auto;

import org.firstinspires.ftc.teamcode.calculators.*;
import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Testing CompleMove", group = "ftc10650")
public class ComplexAuto extends ComplexOp {


    @Override
    public MoveData.StartData startPositionAndOrientation() {
        return new MoveData.StartData(new Vector2D(0,0), 0.0);
    }


    @Override
    public void body() throws InterruptedException {
        d.robot.gyro.resetYaw();
        ComplexMove(
                SpeedCalcs.SetSpeed(.1),

                MotionCalcs.pointSplineMotion(.95,
                        new Vector2D(0.0, 5.0)
//                        new Vector2D(-50.0, 50.0),
//                        new Vector2D(-50.0, 0.0),
//                        new Vector2D(0.0, 0.0)
                        /*,
                        new Vector2D(0, 0)*/),
                OrientationCalcs.holdHeading()
//                OrientationCalcs.spinToProgress(
//                        new OrientationCalcs.spinProgress(0.15, 0.2, 90),
//                        new OrientationCalcs.spinProgress(0.75, 0.85, 0))
                /*OtherCalcs.DistanceStop(OtherCalcs.Side.LEFT,150,145,0.95,1)*/);

//        ComplexMove(
//                SpeedCalcs.SetProgressSpeed(
//                        new SpeedCalcs.ProgressSpeed(1,0.2, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(1,0.4, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.75,0.95, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                MotionCalcs.PointMotion(5,
//                        new Vector2D(-150, 150),
//                        new Vector2D(-300, 0),
//                        new Vector2D(-150, -150),
//                        new Vector2D(0, 0)),
//                OrientationCalcs.spinToProgress(
//                        new OrientationCalcs.spinProgress(0.1, 0.2, 180),
//                        new OrientationCalcs.spinProgress(0.75, 0.85, -90)),
//                OtherCalcs.DistanceStop(OtherCalcs.Side.BACK,1,0,0.95,1));

 
    }
}
