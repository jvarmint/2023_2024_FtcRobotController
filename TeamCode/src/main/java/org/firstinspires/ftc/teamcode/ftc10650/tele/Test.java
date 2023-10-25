package org.firstinspires.ftc.teamcode.ftc10650.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calculators.Interfaces;
import org.firstinspires.ftc.teamcode.calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;

@Disabled
@TeleOp(name = "Test")
public class Test extends ComplexOp {
    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(), 0);
    }

    @Override
    public void body() throws InterruptedException {
        ComplexMove(
                SpeedCalcs.JoystickSpeed(),
                MotionCalcs.ObjectCentricJoystick(),
                OrientationCalcs.turnWithJoystick()
        );
    }
}
