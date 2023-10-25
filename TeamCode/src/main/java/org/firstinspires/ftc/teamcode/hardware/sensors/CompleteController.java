package org.firstinspires.ftc.teamcode.hardware.sensors;

/**
 * Created by Varun on 11/16/2017.
 * Refactored and Edited by Aidan on 3/14/2020.
 */


import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;
import org.firstinspires.ftc.teamcode.utilities.MathUtil;

public class CompleteController {
    Gamepad gamepad;
    private class StickConfig {
        JoystickDeadzoneShape ds;
        JoystickShape js;
        private boolean reverseX;
        private boolean reverseY;
        public double JoystickDeadzoneMag;

        StickConfig(JoystickDeadzoneShape ds, JoystickShape js, boolean reverseX, boolean reverseY, double JoystickDeadzoneMag){
            this.ds = ds;
            this.js = js;
            this.reverseX = reverseX;
            this.reverseY = reverseY;
            this.JoystickDeadzoneMag = JoystickDeadzoneMag;
        }
    }

    StickConfig rightStick;
    StickConfig leftStick;

    private class TrigConfig {
        private double TD;
        private boolean RT;
        TrigConfig(double triggerDeadzone, boolean reverseTrigger){
            this.TD = triggerDeadzone;
            this.RT = reverseTrigger;
        }
    }

    TrigConfig rightTrig;
    TrigConfig leftTrig;

    public CompleteController(Gamepad gamepad){
        this.gamepad = gamepad;
        rightStick = new StickConfig(JoystickDeadzoneShape.CIRCULAR,JoystickShape.CIRCULAR,false,true,0.001);
        leftStick = new StickConfig(JoystickDeadzoneShape.CIRCULAR,JoystickShape.CIRCULAR,false,true,0.001);
        rightTrig = new TrigConfig(0.001,false);
        leftTrig = new TrigConfig(0.001,false);

    }

    public enum JoystickDeadzoneShape{
        CIRCULAR,
        SQUARE
    }

    public enum JoystickShape{
        CIRCULAR,
        SQUARE
    }

    public enum ChoseSide {
        LEFT,
        RIGHT
    }


    public void setJoystick (ChoseSide choice, JoystickShape js, JoystickDeadzoneShape ds, double JDM, boolean reverseX, boolean reverseY){
        switch(choice){
            case LEFT:  leftStick = new StickConfig(ds,js,reverseX,reverseY,JDM);  break;
            case RIGHT: rightStick = new StickConfig(ds,js,reverseX,reverseY,JDM); break;
        }

    }


    public void setTrigger (ChoseSide choice, double TriggerDeadzone, boolean ReverseTrigger){
        switch(choice){
            case LEFT:  leftTrig = new TrigConfig(TriggerDeadzone, ReverseTrigger);  break;
            case RIGHT: rightTrig = new TrigConfig(TriggerDeadzone, ReverseTrigger); break;
        }
    }

    public boolean getButton(String str){
        switch (str){
            case "lsb":
                return gamepad.left_stick_button;

            case "rsb":
                return gamepad.right_stick_button;

            case "lb":
                return gamepad.left_bumper;

            case "rb":
                return gamepad.right_bumper;

            case "u":
                return gamepad.dpad_up;

            case "d":
                return gamepad.dpad_down;

            case "l":
                return gamepad.dpad_left;

            case "r":
                return gamepad.dpad_right;

            case "a":
                return gamepad.a;

            case "b":
                return gamepad.b;

            case "x":
                return gamepad.x;

            case "y":
                return gamepad.y;

            case "back":
                return gamepad.back;

            case "start":
                return gamepad.start;

            case "guide":
                return gamepad.guide;

            default:
                throw new RuntimeException("Not available choice");
        }
    }

    public Vector2D ls(){
        return getControllerJoystick(gamepad.left_stick_x,gamepad.left_stick_y,leftStick);
    }

    public Vector2D rs(){
        return getControllerJoystick(gamepad.right_stick_x,gamepad.right_stick_y,rightStick);
    }

    public double lt(){
        return getControllerTrigger(gamepad.left_trigger,leftTrig);
    }

    public double rt(){
        return getControllerTrigger(gamepad.right_trigger,rightTrig);
    }

    public boolean lsb(){
        return gamepad.left_stick_button;
    }

    public boolean rsb(){
        return gamepad.right_stick_button;
    }

    public boolean lb(){
        return gamepad.left_bumper;
    }

    public boolean rb(){
        return gamepad.right_bumper;
    }

    public boolean u(){
        return gamepad.dpad_up;
    }

    public boolean d(){
        return gamepad.dpad_down;
    }

    public boolean l(){
        return gamepad.dpad_left;
    }

    public boolean r(){
        return gamepad.dpad_right;
    }

    public boolean a(){
        return gamepad.a;
    }

    public boolean b(){
        return gamepad.b;
    }

    public boolean x(){
        return gamepad.x;
    }

    public boolean y(){
        return gamepad.y;
    }

    public boolean back() {
        return gamepad.back;
    }

    public boolean start() {
        return gamepad.start;
    }

    public boolean guide() {
        return gamepad.guide;
    }

    @Deprecated
    public boolean isConnected(){/** THIS DOESN'T WORK DO NOT USE */
        return gamepad.getUser() != null;
    }


    private Vector2D getControllerJoystick(double X, double Y, StickConfig s){
        Vector2D XY = new Vector2D();
        boolean OutsideJoystickDeadzone = false;
        double JDM = s.JoystickDeadzoneMag;
        if(s.reverseX) X=-X;
        if(s.reverseY) Y=-Y;
        switch(s.ds){
            case CIRCULAR:
                OutsideJoystickDeadzone = !(JDM > MathUtil.Distance(X, Y, 0, 0));
                break;

            case SQUARE:
                OutsideJoystickDeadzone = !(X < JDM) && !(Y < JDM);
                break;
        }
        if (OutsideJoystickDeadzone) {
            switch(s.js){

                case CIRCULAR:  XY.x = X;
                                XY.y = Y;
                                break;

                //Square joystick shape means that the maximum output of a joystick lies on a square that fits inside the circle
                //This allows for strafing diagonally at maximum speed
                case SQUARE:    XY.x = X * Math.sqrt(2);
                                XY.y = Y * Math.sqrt(2);

                                if (XY.x > 1 || XY.x < -1) XY.x = MathUtil.getSign(XY.x);
                                if (XY.y > 1 || XY.y < -1) XY.y = MathUtil.getSign(XY.y);

                                break;
            }
        }
        return XY;
    }


    private double getControllerTrigger(double TriggerPos, TrigConfig t){
        double rval;
        if(TriggerPos >= t.TD){
            rval = TriggerPos-t.TD;
        } else {
            rval = 0;
        }
        rval = rval/(1-t.TD);
        if(t.RT) {
            rval = 1 - rval;
        }

        return rval;
    }
}