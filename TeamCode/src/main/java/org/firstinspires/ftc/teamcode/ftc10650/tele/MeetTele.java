package org.firstinspires.ftc.teamcode.ftc10650.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calculators.*;
import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;
import org.firstinspires.ftc.teamcode.utilities.Vector3D;

import java.util.Vector;
@Disabled
@TeleOp(name = "Michael tele", group = "ftc10650")
public class MeetTele extends ComplexOp {

    Vector<SpeedCalcs.ProgressSpeed> s = new Vector<SpeedCalcs.ProgressSpeed>();
    Vector<Vector3D> p = new Vector<Vector3D>();

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(50, 50), 0.0);
    }

    @Override
    public void initMove() throws InterruptedException {

    }

    @Override
    public void body() throws InterruptedException {

        d.initialLiftPos = d.robot.lift.getCurrentPosition();

        RobotMap.camera.setPipeline(d.robot.leftPoleAlignPipeline);

        ComplexMove(
//                null,
//null,
//null,
//                SpeedCalcs.SetSpeed(1.0),
               SpeedCalcs.JoystickSpeed(),
//                MotionCalcs.ObjectCentricJoystick(),
                MotionCalcs.FieldCentricJoystick(0.0),
                //MotionCalcs.ConstantDistanceToPoint(100, new Vector2D(100,100)),
                OrientationCalcs.turnWithJoystick(),
                OtherCalcs.Lift()
//                OtherCalcs.Claw()

                );
                /*OrientationCalcs.lookToPointTurnWithBumperTurnWithJoystick(
                        "a",
                        new OrientationCalcs.lookProgress(new Vector2D(0,0),0.95),
                        new OrientationCalcs.lookProgress(new Vector2D(150,150),1.0)),*/
        /**
         * OtherCalcs.armPath(speeds... , points...
         */
    }
}