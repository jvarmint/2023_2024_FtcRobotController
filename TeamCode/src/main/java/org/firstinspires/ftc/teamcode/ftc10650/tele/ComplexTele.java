package org.firstinspires.ftc.teamcode.ftc10650.tele;//package org.firstinspires.ftc.teamcode.ftc10650.Tele;
//
//import org.firstinspires.ftc.teamcode.Calculators.*;
//import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData;
//import org.firstinspires.ftc.teamcode.Op.ComplexOp;
//import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
//@TeleOp(name = "ComplexTele", group = "ftc10650")
//public class ComplexTele extends ComplexOp {
//
//    @Override
//    public MoveData.StartData startPositionAndOrientation() {
//        return new MoveData.StartData(new Vector2D(0,0), 0);
//    }
//
//    @Override
//    public void body() throws InterruptedException {
//        ComplexMove(null, null, null, OtherCalcs.ResetWobble());
//        ComplexMove(
////                SpeedCalcs.SetSpeed(1.0),
//                SpeedCalcs.JoystickSpeed(),
//                MotionCalcs.FieldCentricJoystick(90),
//                //MotionCalcs.ConstantDistanceToPoint(100, new Vector2D(100,100)),
//                //OrientationCalcs.turnWithJoystick(),
////                OrientationCalcs.lookToOrientationUnderJoystick(0),
////                OrientationCalcs.lookToGoal(),
//                OrientationCalcs.lookToPower(),
//                /*OrientationCalcs.lookToPointTurnWithBumperTurnWithJoystick(
//                        "a",
//                        new OrientationCalcs.lookProgress(new Vector2D(0,0),0.95),
//                        new OrientationCalcs.lookProgress(new Vector2D(150,150),1.0)),*/
////                OtherCalcs.TeleOpMatch(),
//                OtherCalcs.TelemetryPosition(),
//                OtherCalcs.Yeetor(),
//                OtherCalcs.Intake(),
//                OtherCalcs.Wobble(),
//                OtherCalcs.GetXOfGoal(),
//                OtherCalcs.GetPowerPositions(),
//                OtherCalcs.ResetWobbleButton(),
//                OtherCalcs.Bucket());
//
//
///*        ComplexMove(
//                SpeedCalcs.SetSpeed(1),
//                MotionCalcs.PointMotion(5, new Vector2D(100,100)),
//                OrientationCalcs.lookToPoint(new OrientationCalcs.lookProgress(new Vector2D(150,150), 1)),
//                OtherCalcs.TapeMeasure());*/
//    }
//}
