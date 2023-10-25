package org.firstinspires.ftc.teamcode.ftc10650.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.calculators.Interfaces;
import org.firstinspires.ftc.teamcode.calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;
import org.firstinspires.ftc.teamcode.utilities.Vector3D;

import java.util.Vector;

@Disabled
@TeleOp(name = "Raw Arm", group = "ftc10650")
public class RawArm extends ComplexOp {

    Vector<SpeedCalcs.ProgressSpeed> s = new Vector<SpeedCalcs.ProgressSpeed>();
    Vector<Vector3D> p = new Vector<Vector3D>();

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(50, 50), 90);
    }

    @Override
    public void body() throws InterruptedException {
        double pCoef = 20;
        double dCoef = 0;
        boolean lastUpU = false;
        boolean lastUpD = false;
        boolean lastUpR = false;
        boolean lastUpL = false;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients();
//        d.telemetry.addData("string", "string");
//        d.robot.tarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        d.robot.barm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive()) {
//            telemetry.addData("string2", "string2");

//            d.robot.tarmEx.setPower(d.manip.rs().x / 5.0);
            if(d.manip.u() && !lastUpU) {
                pCoef += 0.5;
            }
            if (d.manip.d() && !lastUpD) {
                pCoef -= 0.5;
            }
            if(d.manip.r() && !lastUpR) {
                dCoef += 0.5;
            }
            if (d.manip.l() && !lastUpL) {
                dCoef -= 0.5;
            }

            lastUpU = d.manip.u();
            lastUpD = d.manip.d();
            lastUpR = d.manip.r();
            lastUpL = d.manip.l();

            pidfCoefficients.p = 20;
            pidfCoefficients.i = 0;
            pidfCoefficients.d = dCoef;
            pidfCoefficients.f = pCoef;
            d.telemetry.addData("p", pCoef);
            d.telemetry.addData("d", dCoef);
            telemetry.update();
            Thread.sleep(10);

        }
        /**
         * OtherCalcs.armPath(speeds... , points...
         */
    }
}
