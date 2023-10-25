package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


public class NavX {

    private static NavX instance = null;
    private NavxMicroNavigationSensor imu;
    public double startAng, startPitch;
    HardwareMap hw;

    public NavX(HardwareMap hw, String name, double startAng){
        this.imu = hw.get(NavxMicroNavigationSensor.class, name);
        this.startAng = startAng;
        this.hw = hw;
    }

    public double getYaw() {

        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAng;

    }

    public double getHeading(){

        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAng;

    }

    public double getYawDeg(){

        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAng;

    }

    public double getYawRad(AngleUnit au){

        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - startAng;

    }

    public double getPitch() {

        return (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle * -1) - startPitch;

    }

    public void resetPitch() {

        startPitch = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;

    }

    public double getRawHeading() {

        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    }

    public void close(){
        imu = null;
        instance = null;
    }

    public double getLatency(){
        return (System.nanoTime() - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).acquisitionTime)/1000000;
    }

    public double getUpdatedYaw(){
        while((System.nanoTime() - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).acquisitionTime)/1000000 > 5){
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getOutput() {
        return getHeading();
    }


}