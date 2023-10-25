package org.firstinspires.ftc.teamcode.ftc10650.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap;


@TeleOp(name = "reset heading")

public class ResetHeading extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        new RobotMap(hardwareMap);
        waitForStart();
        RobotMap.gyro.resetYaw();
    }
}
