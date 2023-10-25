package org.firstinspires.ftc.teamcode.ftc10650.auto;//package org.firstinspires.ftc.teamcode.ftc10650.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData;
//import org.firstinspires.ftc.teamcode.Calculators.MotionCalcs;
//import org.firstinspires.ftc.teamcode.Calculators.OrientationCalcs;
//import org.firstinspires.ftc.teamcode.Calculators.OtherCalcs;
//import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;
//import org.firstinspires.ftc.teamcode.Op.ComplexOp;
//import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
//
//@Disabled
//@Autonomous(name = "Meet 1 Auto (Left)", group = "ftc10650")
//public class Meet1Auto2022LEFT extends ComplexOp {
//
//
//    @Override
//    public MoveData.StartData startPositionAndOrientation() {
//        return new MoveData.StartData(new Vector2D(0,0), 0);
//    }
//
//    @Override
//    public void body() throws InterruptedException {
//        d.startHeading = (double) d.robot.gyro.getAngularOrientation().firstAngle;
//        ComplexMove(null, null, null, OtherCalcs.SignalPipeline(5000));
//        int Signal = d.signalColor;
////        Signal = 2;
//        switch (Signal){
//            case 1:
//                d.robot.fleftEx.setVelocity(-200);
//                d.robot.frightEx.setVelocity(200);
//                d.robot.bleftEx.setVelocity(200);
//                d.robot.brightEx.setVelocity(-200);
//                sleep(2500);
//                d.robot.fleftEx.setVelocity(0);
//                d.robot.frightEx.setVelocity(0);
//                d.robot.bleftEx.setVelocity(0);
//                d.robot.brightEx.setVelocity(0);
//                sleep(1000);
//                d.robot.fleftEx.setVelocity(200);
//                d.robot.frightEx.setVelocity(200);
//                d.robot.bleftEx.setVelocity(200);
//                d.robot.brightEx.setVelocity(200);
//                sleep(2500);
//                d.robot.fleftEx.setVelocity(0);
//                d.robot.frightEx.setVelocity(0);
//                d.robot.bleftEx.setVelocity(0);
//                d.robot.brightEx.setVelocity(0);
//                break;
//
//            case 2:
//                d.robot.fleftEx.setVelocity(250);
//                d.robot.frightEx.setVelocity(300);
//                d.robot.bleftEx.setVelocity(300);
//                d.robot.brightEx.setVelocity(250);
//                sleep(5000);
//                d.robot.fleftEx.setVelocity(0);
//                d.robot.frightEx.setVelocity(0);
//                d.robot.bleftEx.setVelocity(0);
//                d.robot.brightEx.setVelocity(0);
//                break;
//            case 3:
//                d.robot.fleftEx.setVelocity(-200);
//                d.robot.frightEx.setVelocity(200);
//                d.robot.bleftEx.setVelocity(-200);
//                d.robot.brightEx.setVelocity(200);
//                sleep(3600);
//                d.robot.fleftEx.setVelocity(0);
//                d.robot.frightEx.setVelocity(0);
//                d.robot.bleftEx.setVelocity(0);
//                d.robot.brightEx.setVelocity(0);
//                sleep(1000);
//                d.robot.fleftEx.setVelocity(350);
//                d.robot.frightEx.setVelocity(-350);
//                d.robot.bleftEx.setVelocity(-350);
//                d.robot.brightEx.setVelocity(350);
//                sleep(3400);
//                d.robot.fleftEx.setVelocity(0);
//                d.robot.frightEx.setVelocity(0);
//                d.robot.bleftEx.setVelocity(0);
//                d.robot.brightEx.setVelocity(0);
//                sleep(1000);
//                d.robot.fleftEx.setVelocity(-350);
//                d.robot.frightEx.setVelocity(-350);
//                d.robot.bleftEx.setVelocity(-350);
//                d.robot.brightEx.setVelocity(-350);
//                sleep(2500);
//                d.robot.fleftEx.setVelocity(0);
//                d.robot.frightEx.setVelocity(0);
//                d.robot.bleftEx.setVelocity(0);
//                d.robot.brightEx.setVelocity(0);
//                break;
//
//
//        }
//
//
//
//        //d.robot.fleft
//        //96mm radius
////        final double circum = 0.603185789489;
////        //(96*2*pi)/1000.
////
////        waitForStart();
////        if(opModeIsActive()){
//////            d.robot.fright.setPower(.125);
//////            d.robot.bleft.setPower(.125);
//////            d.robot.bright.setPower(.125);
////            switch(Signal){
//////                case 1:
//////
//////                    d.robot.fleft.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.fright.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.bleft.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.bright.setTargetPosition((int)(360.*circum/.6));
//////
//////                    d.robot.fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////
//////                    sleep(500);
//////
//////                    d.robot.fleft.setTargetPosition((int)(-360.*circum/.6));
//////                    d.robot.fright.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.bleft.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.bright.setTargetPosition((int)(-360.*circum/.6));
//////
//////                    d.robot.fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////
//////                    break;
//////                case 2:
//////                    d.robot.fleft.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.fright.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.bleft.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.bright.setTargetPosition((int)(360.*circum/.6));
//////
//////                    d.robot.fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////
//////                    break;
//////                case 3:
//////                    d.robot.fleft.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.fright.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.bleft.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.bright.setTargetPosition((int)(360.*circum/.6));
//////
//////                    d.robot.fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////
//////                    sleep(500);
//////
//////                    d.robot.fleft.setTargetPosition((int)(360.*circum/.6));
//////                    d.robot.fright.setTargetPosition((int)(-360.*circum/.6));
//////                    d.robot.bleft.setTargetPosition((int)(-360.*circum/.6));
//////                    d.robot.bright.setTargetPosition((int)(360.*circum/.6));
//////
//////                    d.robot.fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    d.robot.bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////                    break;
////                case 1:
////                    d.robot.fleftEx.setVelocity(200);
////                    d.robot.frightEx.setVelocity(200);
////                    d.robot.bleftEx.setVelocity(200);
////                    d.robot.brightEx.setVelocity(200);
////                    sleep(1800);
////                    d.robot.fleftEx.setVelocity(0);
////                    d.robot.frightEx.setVelocity(0);
////                    d.robot.bleftEx.setVelocity(0);
////                    d.robot.brightEx.setVelocity(0);
////                    sleep(1000);
////                    d.robot.fleftEx.setVelocity(-200);
////                    d.robot.frightEx.setVelocity(200);
////                    d.robot.bleftEx.setVelocity(200);
////                    d.robot.brightEx.setVelocity(-200);
////                    sleep(1200);
////                    d.robot.fleftEx.setVelocity(0);
////                    d.robot.frightEx.setVelocity(0);
////                    d.robot.bleftEx.setVelocity(0);
////                    d.robot.brightEx.setVelocity(0);
////                    /*
////                                        d.robot.fleftEx.setPower(.2);
////                    d.robot.frightEx.setPower(.22);
////                    d.robot.bleftEx.setPower(.2);
////                    d.robot.brightEx.setPower(.22);
////                    sleep(1800);
////                    d.robot.frightEx.setPower(0);
////                    d.robot.bleftEx.setPower(0);
////                    d.robot.brightEx.setPower(0);
////                    sleep(1000);
////                    d.robot.fleftEx.setPower(-.3);
////                    d.robot.frightEx.setPower(.33);
////                    d.robot.bleftEx.setPower(.3);
////                    d.robot.brightEx.setPower(-.3);
////                    sleep(1200);
////                    d.robot.fleftEx.setPower(0);
////                    d.robot.frightEx.setPower(0);
////                    d.robot.bleftEx.setPower(0);
////                    d.robot.brightEx.setPower(0);
////                     */
////
////                    break;
////                case 2:
////                    d.robot.fleft.setPower(.2);
////                    d.robot.fright.setPower(.22);
////                    d.robot.bleft.setPower(.2);
////                    d.robot.bright.setPower(.22);
////                    sleep(465);
////                    d.robot.fleft.setPower(0);
////                    d.robot.fright.setPower(0);
////                    d.robot.bleft.setPower(0);
////                    d.robot.bright.setPower(0);
////                    sleep(1000);
////                    d.robot.fleft.setPower(-.3);
////                    d.robot.fright.setPower(.33);
////                    d.robot.bleft.setPower(.3);
////                    d.robot.bright.setPower(-.3);
////                    sleep(100);
////                    d.robot.fleft.setPower(0);
////                    d.robot.fright.setPower(0);
////                    d.robot.bleft.setPower(0);
////                    d.robot.bright.setPower(0);
////                    break;
////
////                case 3:
////                    d.robot.fleft.setPower(.2);
////                    d.robot.fright.setPower(.22);
////                    d.robot.bleft.setPower(.2);
////                    d.robot.bright.setPower(.22);
////                    sleep(465);
////                    d.robot.fleft.setPower(0);
////                    d.robot.fright.setPower(0);
////                    d.robot.bleft.setPower(0);
////                    d.robot.bright.setPower(0);
////                    sleep(1000);
////                    d.robot.fleft.setPower(.3);
////                    d.robot.fright.setPower(-.33);
////                    d.robot.bleft.setPower(-.3);
////                    d.robot.bright.setPower(.3);
////                    sleep(400);
////                    d.robot.fleft.setPower(0);
////                    d.robot.fright.setPower(0);
////                    d.robot.bleft.setPower(0);
////                    d.robot.bright.setPower(0);
////                    break;
////
////            }
////        }
//
//
//
////        ComplexMove(
////                SpeedCalcs.SetSpeed(.2),
////                MotionCalcs.PointMotion(0.1,
////                        new Vector2D(50, 0),
////                        new Vector2D(50, 50)),
////                OrientationCalcs.holdHeading());
////
////
////        ComplexMove(
////                SpeedCalcs.SetSpeed(1),
////                MotionCalcs.PointMotion(1,
////                        new Vector2D(-80, 90),
////                        new Vector2D(-80, 280),
////                        new Vector2D(-25, 260),
////                        new Vector2D(-75, 75),
////                        new Vector2D(-50, 260)
////                        /*,
////                        new Vector2D(0, 0)*/),
////                OrientationCalcs.spinToProgress(
////                        new OrientationCalcs.spinProgress(0.0, 0.1, 90),
////                        new OrientationCalcs.spinProgress(0.1, 0.2, 0)),
////                OtherCalcs.AutoOpMatch()
////                /*OtherCalcs.DistanceStop(OtherCalcs.Side.LEFT,150,145,0.95,1)*/);
////
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
////                OtherCalcs.DistanceStop(OtherCalcs.Side.FRONT,15,10,0.95,1),
////                OtherCalcs.AutoOpMatch());
//
//
//    }
//}
