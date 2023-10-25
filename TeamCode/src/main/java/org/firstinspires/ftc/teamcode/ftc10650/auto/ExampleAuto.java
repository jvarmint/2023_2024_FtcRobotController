package org.firstinspires.ftc.teamcode.ftc10650.auto;

import org.firstinspires.ftc.teamcode.calculators.*;
import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "ExampleAuto", group = "ftc10650")
public class ExampleAuto extends LinearOpMode {
    public Interfaces.MoveData d = new Interfaces.MoveData();

    @Override
    public void runOpMode() throws InterruptedException {
        d.robot = new RobotMap(hardwareMap);//, startPositionAndOrientation());
        waitForStart();

        d.robot.bleft.setPower(0.3);
        d.robot.fleft.setPower(0.3);
        d.robot.bright.setPower(0.3);
        d.robot.fright.setPower(0.3);
        Thread.sleep(1200);
        d.robot.bleft.setPower(0.0);
        d.robot.fleft.setPower(0.0);
        d.robot.bright.setPower(0.0);
        d.robot.fright.setPower(0.0);

    }
}
