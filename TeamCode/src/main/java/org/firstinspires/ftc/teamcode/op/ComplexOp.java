package org.firstinspires.ftc.teamcode.op;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.calculators.OtherCalcs;
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera;
import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap;
import org.firstinspires.ftc.teamcode.utilities.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.calculators.Interfaces;
import org.firstinspires.ftc.teamcode.hardware.sensors.CompleteController;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

public abstract class ComplexOp extends LinearOpMode{

    private MecanumDrive mecanumDrive;

    double previousHeading = 0;

//    private Vector2D slamraOffset = new Vector2D(-2.0, -16.0);

//    private Pose2d initPose = null;
//    private double initPoseX = 0.0;
//    private double initPoseY = 0.0;

    public void ComplexMove(Interfaces.SpeedCalc speedCalc,
                            Interfaces.MotionCalc motionCalc,
                            Interfaces.OrientationCalc orientationCalc,
                            Interfaces.OtherCalc... otherCalc) throws InterruptedException {

        d.progress = 0;

        Vector2D vector = new Vector2D();

        float endGameTime = 0;

        d.lastCommand = d.currentCommand;
        d.currentCommand = new Interfaces.MoveData.Command(0, vector,0.0);


//        DatagramSocket ds = null;
//        try {
//            ds = new DatagramSocket();
//        } catch (SocketException e) {
//            e.printStackTrace();
//        }


        while(d.progress < 1.0) {
            //_______________________


//            if(ds != null) {
//                InetAddress ip = null;
//                try {
//                    ip = InetAddress.getByName("192.168.43.255");
////                    ip = InetAddress.getLocalHost();
//                } catch (UnknownHostException e) {
//                    e.printStackTrace();
//                }
//                String str = String.valueOf(d.robot.bleft.getCurrentPosition());
//                byte[] strBytes = str.getBytes();
//                DatagramPacket DpSend =
//                        new DatagramPacket(strBytes, strBytes.length, ip, 10650);
//                try {
//                    ds.send(DpSend);
//                    telemetry.addData("datagram: ", DpSend);
//                } catch (IOException e) {
//                    e.printStackTrace();
//                }
//            }


            //_______________________
            telemetry.addData("lift position", d.robot.lift.getCurrentPosition());


//            Orientation orientation = d.robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//            double heading = -orientation.thirdAngle-d.startData.StartNorthOffset;
//            Orientation orientation = d.robot.gyro.getAngularOrientation();//OLD INTERFACE

//            double heading = orientation.firstAngle-d.startHeading-d.startData.StartNorthOffset;
            YawPitchRollAngles robotOrientation = d.robot.gyro.getRobotYawPitchRollAngles();
            double heading = robotOrientation.getYaw(AngleUnit.DEGREES);
            double diffHeading = heading - previousHeading;
            telemetry.addData("raw heading", heading);
//            telemetry.addData("d.startHeading", d.startHeading);
            telemetry.addData("d.StartNorthOffset", d.startData.StartNorthOffset);
            telemetry.addData("firstAngle", robotOrientation.getYaw(AngleUnit.DEGREES));


            if(diffHeading > 180.0) {
                diffHeading -= 360.0;
            } else if (diffHeading <= -180.0){
                diffHeading += 360.0;
            }
            d.heading += diffHeading;
            previousHeading = heading;
            telemetry.addData("d.heading", d.heading);
//
//            telemetry.addData("gryo", orientation.thirdAngle);
//            telemetry.addData("orientation", d.heading);

            double distanceCorrectionFactorForward = 0.019;
            double distanceCorrectionFactorSide = 0.01864;
            Vector2D encoderPre = d.encoderPos.clone();
            long lastEncoderUpdateTime = d.encodePosUpdateTimeMillis;
            d.encoderPos = mecanumDrive.getVectorDistanceCm();
            telemetry.addData("encoderPos",d.encoderPos);
            d.encodePosUpdateTimeMillis = System.currentTimeMillis();
            Vector2D deltaMove = d.encoderPos.getSubtracted(encoderPre);
            double unitsForwardPerTile = 85.0/4.0; //135.0/4.0;
            double unitsStrafePerTile = 99.5/4.0; //156.0/4.0;
            deltaMove.x /= unitsStrafePerTile;
            deltaMove.y /= unitsForwardPerTile;
            Vector2D moveSpeed = deltaMove.getDivided(Math.max(0.001,(d.encodePosUpdateTimeMillis - lastEncoderUpdateTime)/1000.0));
//
            deltaMove.rotateBy(Math.toRadians(d.heading));//WAS -d.heading !!!!!!!!!!!!!!!!!!!!//180+d.heading
            d.preWPos.set(d.wPos);
            d.wPos.add(deltaMove);


            if(orientationCalc != null) d.currentCommand.orientationSpeed = orientationCalc.CalcOrientation(d);
            if(motionCalc != null) {
                d.currentCommand.motionSpeed = motionCalc.CalcMotion(d).clone();
                d.currentCommand.motionSpeed.rotateBy(Math.toRadians(-d.heading));
            }
            if(speedCalc != null) d.currentCommand.speed = speedCalc.CalcSpeed(d);

            for (Interfaces.OtherCalc calc : otherCalc) calc.CalcOther(d);

            if (d.timeRemainingUntilEndgame >= 0) endGameTime = (float)(Math.round(d.timeRemainingUntilEndgame / 100) / 10.0);

            telemetry.addData("Robot is here", "\n"+d.field);
            telemetry.addData("Position", d.wPos.x + "   " + d.wPos.y);
            telemetry.addData("Left Driver Joystick", d.driver.ls());
            telemetry.addData("bleft velocity", d.robot.bleftEx.getVelocity());
            telemetry.addData("fleft velocity", d.robot.fleftEx.getVelocity());
            telemetry.addData("bright velocity", d.robot.brightEx.getVelocity());
            telemetry.addData("fright velocity", d.robot.frightEx.getVelocity());
            telemetry.addData("signal position", d.debugData1);
            telemetry.addData("right joystick", d.driver.rs());
            telemetry.update();


            mecanumDrive.driveMecanum(
                    d.currentCommand.motionSpeed.getAdded(d.robotCentricAdditiveVector),
                    d.currentCommand.speed + d.robotCentricAdditiveVector.getLength(),
                    d.currentCommand.orientationSpeed);



            d.progress = MathUtil.findMaxList(
                    motionCalc == null ? 0 : motionCalc.myProgress(d),
                    orientationCalc == null ? 0 : orientationCalc.myProgress(d),
                    speedCalc == null ? 0 : speedCalc.myProgress(d));


            for (Interfaces.OtherCalc calc : otherCalc) d.progress = Math.max(d.progress,calc.myProgress(d));
            telemetry.addData("Progress", d.progress);

            Thread.sleep(10);


            if (!opModeIsActive()) throw new InterruptedException();
        }
        d.robotCentricAdditiveVector = new Vector2D();
    }

    //How data is transferred between calculators and complexOp
    public Interfaces.MoveData d = new Interfaces.MoveData();//if you delete this the world will end


    void initHardware(HardwareMap hwMap) {

        telemetry.addData("ENTERED INIT HARDWARE", "<-");
        d.telemetry = telemetry;
        d.robot = new RobotMap(hwMap);//, startPositionAndOrientation());
//        d.robot.slamra.setPose(new Pose2d(0, 0,
////                startPositionAndOrientation().StartPos.x/100,
////                startPositionAndOrientation().StartPos.y/100,
//
//                new Rotation2d(0)));//Math.toRadians(startPositionAndOrientation().StartHeading))));
//        d.robot.slamra.start();



        mecanumDrive = new MecanumDrive(d);
    }

    public abstract Interfaces.MoveData.StartData startPositionAndOrientation();

    public abstract void body() throws InterruptedException;

    public void initMove() throws InterruptedException{

    }

//    public void saveInitialHeading(boolean forceSave){
//        if(forceSave || !d.startHeadingIsSet){
//            d.startHeading = d.robot.gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            d.startHeadingIsSet = true;
//        }
//    }

    void exit(){//so we don't run into a wall at full speed
//        d.lastFrameBarmPos = d.robot.barm.getCurrentPosition();
//        d.lastFrameTarmPos = d.robot.tarm.getCurrentPosition();
//        d.lastFrameSarmPos = d.robot.sarm.getCurrentPosition();
//        d.initBarmPos = d.initBarmPos - d.robot.barm.getCurrentPosition();// - d.firstFrameBarmPos ;
//        d.initTarmPos = d.initTarmPos - d.robot.tarm.getCurrentPosition();// - d.firstFrameTarmPos ;
//        d.initSarmPos = d.initSarmPos - d.robot.sarm.getCurrentPosition();// - d.firstFrameSarmPos ;

        d.robot.bright.setPower(0);
        d.robot.fright.setPower(0);
        d.robot.bleft.setPower(0);
        d.robot.fleft.setPower(0);
//        d.robot.tarm.setPower(0);
//        d.robot.barm.setPower(0);
//        d.robot.sarm.setPower(0);

//        d.robot.slamra.stop();

    }



    @Override
    public void runOpMode() throws InterruptedException{

        //INITIALIZATION
        telemetry.addData("Initializing", "Started");
        telemetry.update();


        d.isFinished = false;
        d.isStarted = false;

        d.driver = new CompleteController(gamepad1);
        d.manip = new CompleteController(gamepad2);

        //
        //
        //try {
        initHardware(hardwareMap);
        //} catch (Exception e){
        //    StringWriter sw = new StringWriter();
        //    PrintWriter pw = new PrintWriter(sw);
        //    e.printStackTrace(pw);
        //    telemetry.addData(sw.toString(), "this");
        //}
        //
        //

        //START POSITION
        if (d.startData == null) {
            d.startData = this.startPositionAndOrientation();
            d.preWPos.set(d.startData.StartPos.clone());
            d.wPos.set(d.startData.StartPos.clone());
        }

        final Interfaces.OtherCalc posDisplay = OtherCalcs.TelemetryPosition();
        posDisplay.CalcOther(d);

        telemetry.addData("Place robot here", "\n"+d.field);
        telemetry.addData("heading"," "+d.startData.StartNorthOffset +" | position: ("+String.valueOf((d.startData.StartPos.x))+", "+String.valueOf((d.startData.StartPos.y))+")");
        telemetry.update();

//        d.firstFrameBarmPos = d.robot.barm.getCurrentPosition();
//        d.firstFrameTarmPos = d.robot.tarm.getCurrentPosition();
//        d.firstFrameSarmPos = d.robot.sarm.getCurrentPosition();
//        d.firstLiftPos = d.robot.lift.getCurrentPosition();




//        saveInitialHeading(true);
        initMove();


        waitForStart();



//        initPose = d.robot.slamra.getLastReceivedCameraUpdate().pose;
//        d.telemetry.addData("confidence", d.robot.slamra.getLastReceivedCameraUpdate().confidence);
//        initPoseX = initPose.getTranslation().getX();
//        initPoseY = initPose.getTranslation().getY();
        d.telemetry.update();

        d.isStarted = true;

        //BODY
        try {
            body();
        } catch (InterruptedException ie) {
            telemetry.addData("Interrupted","Exception");
            telemetry.update();
        }
        telemetry.addData("Body", "Finished");
        telemetry.update();

        //EXIT
        telemetry.addData("Exit", "Started");
        telemetry.update();
        exit();
        d.isFinished = true;
        telemetry.addData("Exit", "Finished");
        telemetry.update();
    }
}
