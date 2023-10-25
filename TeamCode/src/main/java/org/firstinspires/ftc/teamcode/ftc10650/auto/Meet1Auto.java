package org.firstinspires.ftc.teamcode.ftc10650.auto;//package org.firstinspires.ftc.teamcode.ftc10650.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData;
//import org.firstinspires.ftc.teamcode.Calculators.MotionCalcs;
//import org.firstinspires.ftc.teamcode.Calculators.OrientationCalcs;
//import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;
//import org.firstinspires.ftc.teamcode.Op.ComplexOp;
//import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
//
//
//@Autonomous(name = "Meet 1 Auto", group = "ftc10650")
//public class Meet1Auto extends ComplexOp {
//
//    @Override
//    public MoveData.StartData startPositionAndOrientation() {
//        return new MoveData.StartData(new Vector2D(150,0), 90);
//    }
//
//    @Override
//    public void initMove() throws InterruptedException {
//
////        s.add(new SpeedCalcs.ProgressSpeed(0.05, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG));
////        s.add(new SpeedCalcs.ProgressSpeed(0.2, 1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG));
////        p.add(new Vector3D(0, 0.3f, 0.3f));
//        d.robot.barm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        d.robot.tarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        while(d.robot.bop.getState()) {
//            d.robot.barm.setPower(-0.2);
//            d.robot.tarm.setPower(-0.08);
//        }
//        d.robot.barm.setPower(0.0);
//        d.robot.barm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        d.robot.barm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        d.initBarmPos = d.robot.barm.getCurrentPosition();
////        d.robot.barm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while(d.robot.top.getState()) {
//            d.robot.tarm.setPower(0.2);
//        }
//        d.robot.tarm.setPower(0.0);
//        d.robot.tarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        d.robot.tarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        d.initTarmPos = d.robot.tarm.getCurrentPosition();
//        d.initSarmPos = d.robot.sarm.getCurrentPosition();
//        while(true){
//            d.arm.update();
//            d.telemetry.update();
//            if(d.arm.moveTowardTarget(-51.04, -0.1215, .090, .1)) break;
//        }
//        d.robot.claw.setPosition(0.65);
//
////        d.robot.tarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    }
//    //-51.04 sarm
//    //51.31 barm
//    //203.4 tarm
//    // x -0.1215 y .0951
//    @Override
//    public void body() throws InterruptedException {
//        d.duckPos = d.robot.duckSpotPipeline.getDuckPos();
//        d.robot.clawCam.setPipeline(d.robot.cubeFindPipeline);
//
//        ComplexMove(
//                SpeedCalcs.SetProgressSpeed(
//                        new SpeedCalcs.ProgressSpeed(0.1,0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.3,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                MotionCalcs.PointMotion(10,
//                        new Vector2D(200, 0),
//                        new Vector2D(200, 50),
//                        new Vector2D(150, 0)
////                        new Vector2D(-300, 0),
////                        new Vector2D(-150, -150),
////                        new Vector2D(0, 0)
//                        /*,
//                        new Vector2D(0, 0)*/),
//                OrientationCalcs.lookToOrientation(0)
//
//
////                OrientationCalcs.spinToProgress(
////                        new OrientationCalcs.spinProgress(0.15, 0.2, 90),
////                        new OrientationCalcs.spinProgress(0.75, 0.85, 0))
//                /*OtherCalcs.DistanceStop(OtherCalcs.Side.LEFT,150,145,0.95,1)*/);
//
////        ComplexMove(
////                SpeedCalcs.SetProgressSpeed(
////                        new SpeedCalcs.ProgressSpeed(1,0.2, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.1,0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(1,0.4, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.75,0.95, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
////                MotionCalcs.PointMotion(5,
////                        new Vector2D(-150, 150),
////                        new Vector2D(-300, 0),
////                        new Vector2D(-150, -150),
////                        new Vector2D(0, 0)),
////                OrientationCalcs.spinToProgress(
////                        new OrientationCalcs.spinProgress(0.1, 0.2, 180),
////                        new OrientationCalcs.spinProgress(0.75, 0.85, -90)),
////                OtherCalcs.DistanceStop(OtherCalcs.Side.BACK,1,0,0.95,1));
//
//
//    }
//}
