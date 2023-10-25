package org.firstinspires.ftc.teamcode.hardware.robertomap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.sensors.pipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.hardware.sensors.pipelines.PoleAlignPipeline;
import org.firstinspires.ftc.teamcode.hardware.sensors.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.hardware.sensors.pipelines.SignalPipelineRight;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class RobotMap {

    public static DcMotor bleft, fleft, bright, fright, lift;

    public static DcMotorEx bleftEx, fleftEx, brightEx, frightEx, liftEx;

    public static Servo claw, pitch;

    public static IMU gyro;

    public static OpenCvCamera camera;

    public final PropDetectionPipeline propPipe = new PropDetectionPipeline();

    public final SignalPipelineRight signalPipelineRight = new SignalPipelineRight();

    public final PoleAlignPipeline leftPoleAlignPipeline = new PoleAlignPipeline(
            new Point(371.0, -700.0),    //high was 380, -700; 17 up
            new Point(320.0, 300.0),    //mid
            new Point(320.0, 500.0),
            1.65, 0, 0, true);   //low

    public final PoleAlignPipeline rightPoleAlignPipeline = new PoleAlignPipeline(
            new Point(149.0, -60.0),      //high (158.5,-60); 17 down
            new Point(158.5, 300.0),    //mid
            new Point(158.5, 500.0),
            1.1, 80, 30, false);   //low

    public final AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(1.0, 578.272, 578.272, 402.145, 221.506);//(0.0375, 40.0, 40.0, 30.0, 20.0);

    public static HardwareMap hw;



    public RobotMap(HardwareMap hw) {

        this.hw = hw;
        /**
         * @see <a href="https://ftc-tricks.com/dc-motors/"</a>
         * @see DcMotor.RunMode.RUN_USING_ENCODER this implements a PID for all of the motors
         * This elminates the problems such as the inconsistent auto and having to charge the battery to full every use
         * @see DcMotorSimple.Direction.REVERSE is the correct place to change the directions of the motors
         * it should not be done in a higher level code this is the correct spot
         */
//PIDCoefficients pidDrive = new PIDCoefficients(50, 10, 0);
        PIDFCoefficients pidDrive = new PIDFCoefficients(3, 2, 1, 20);//p5 i2 d5 f17.5
//        PIDFCoefficients pidDrive = new PIDFCoefficients(10, 4, 1, 10);//p5 i2 d5 f17.5

        bleft = hw.get(DcMotor.class, "bleft");
        bleft.setDirection(DcMotorSimple.Direction.FORWARD);
        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bleftEx = (DcMotorEx) bleft;
        bleftEx.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);

        fleft = hw.get(DcMotor.class, "fleft");
        fleft.setDirection(DcMotorSimple.Direction.FORWARD);
        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fleftEx = (DcMotorEx) fleft;
        fleftEx.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);

        bright = hw.get(DcMotor.class, "bright");
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brightEx = (DcMotorEx) bright;
        brightEx.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);

        fright = hw.get(DcMotor.class, "fright");
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frightEx = (DcMotorEx) fright;
        frightEx.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);

//
//        lift = hw.get(DcMotor.class, "lift");
//        lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setTargetPosition(lift.getCurrentPosition());
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        liftEx = (DcMotorEx) lift;
//        frightEx.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidDrive);

//
//        claw = hw.get(Servo.class, "claw");
//        claw.setPosition(.23);
//
//        pitch = hw.get(Servo.class, "pitch");
//        pitch.setPosition(0.75);



//        gyro = hw.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//
//        gyro.initialize(parameters);

        gyro = hw.get(IMU.class, "imu");
        gyro.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

//        CameraManager cameraManager = ClassFactory.getInstance().getCameraManager();
//        List<WebcamName> webcams = cameraManager.getAllWebcams();
//        RobotLog.a("CAMERA TEST MESSAGE");
//        for(WebcamName w:webcams){
//            RobotLog.aa("CAMERA DEVICE NAME", w + " : "+w.getSerialNumber().toString()+" : "+w.getDeviceName().toString()+" : "+w.getManufacturer()+" : "+w.getVersion());
//
//        }
//        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().sp

        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId,
                        2,
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hw.get(WebcamName.class, "CAMERA"),
                viewportContainerIds[1]);

//        OpenCvCameraFactory.getInstance().createWebcam(
//                hw.get(WebcamName.class, "LeftCamera"),
//                viewportContainerIds[0]).getExposureControl().setExposure(100, TimeUnit.MILLISECONDS);


        camera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        camera.startStreaming(640, 360, OpenCvCameraRotation.UPSIDE_DOWN);
                        camera.setPipeline(propPipe);
//                        leftCamera.setPipeline(leftPoleAlignPipeline);
//                        leftCamera.setPipeline(signalPipeline);
                        camera.showFpsMeterOnViewport(true);
                    }

                    @Override
                    public void onError(int errorCode) {

                    }
                }
        );


    }
}
