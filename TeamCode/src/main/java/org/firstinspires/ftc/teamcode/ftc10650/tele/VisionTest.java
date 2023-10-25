package org.firstinspires.ftc.teamcode.ftc10650.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData.StartData;
import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;

//import kotlin.Math.sign
@Disabled
@TeleOp(name = "vision test", group = "ftc10650")
public class VisionTest extends ComplexOp {


    @Override
    public StartData startPositionAndOrientation() {
        return new StartData(new Vector2D(), 0);
    }

    @Override
    public void body() throws InterruptedException {
//        RobotMap.leftCamera.setPipeline(d.robot.aprilTagDetectionPipeline);
        ComplexMove(null, null, null);
    }
}