package org.firstinspires.ftc.teamcode.calculators;

import org.firstinspires.ftc.teamcode.utilities.*;

public class SpeedCalcs {


    public static Interfaces.SpeedCalc SetSpeed(final double desiredSpeed){

        return new Interfaces.SpeedCalc(){
            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }

            @Override
            public double CalcSpeed(Interfaces.MoveData d){
                return desiredSpeed;
            }
        };
    }


    public static Interfaces.SpeedCalc JoystickSpeed(){
        return  new Interfaces.SpeedCalc() {
            @Override
            public double CalcSpeed(Interfaces.MoveData d) {
                if(d.driver.rt()>.5) {
                    return d.driver.ls().getLength();
                }
                return d.driver.ls().getLength()/4.0;
//                return Math.pow(d.driver.ls().getLength(),2);
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }

    public static Interfaces.SpeedCalc StandardRampUpDown(double minSpeed, double maxSpeed, double reachMaxByProgress){
        return SpeedCalcs.SetProgressSpeed(
                new ProgressSpeed(minSpeed, 0.0, ProgressSpeed.timeOrProg.PROG),

                new ProgressSpeed(maxSpeed, reachMaxByProgress, ProgressSpeed.timeOrProg.PROG),

                new ProgressSpeed(maxSpeed, 1.0-reachMaxByProgress, ProgressSpeed.timeOrProg.PROG),

                new ProgressSpeed(minSpeed, 1.0, ProgressSpeed.timeOrProg.PROG)
        );
    }


    public static Interfaces.SpeedCalc SetProgressSpeed(final ProgressSpeed... progressSpeed){

        return new Interfaces.SpeedCalc(){
            private double desiredSpeed = 0.1;//set this to minimum moving speed
            private boolean timeStart = false;
            private double thisProgress = 0;
            private int currIndex = -1;
            private double startingSpeed = 0;
            private double startingProg = 0;
            private boolean nextSpeed = false;
            TimeUtil timeUtil = new TimeUtil();
            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }

            @Override
            public double CalcSpeed(Interfaces.MoveData d){
                if(currIndex<0){
                    timeStart = false;
                    startingSpeed = d.previousSpeed;
                    startingProg = 0;
                    currIndex = 0;
                }
                if(currIndex < progressSpeed.length){
                    ProgressSpeed current = progressSpeed[currIndex];
                    //                for(int i=0;i<progressSpeed .length;i++){

                    switch(current.timeProg){
                        case PROG:
                            desiredSpeed = MathUtil.map(d.progress,startingProg,current.atDimension, startingSpeed,current.rampToSpeed);
                            nextSpeed = (d.progress>=current.atDimension);
                            if (nextSpeed) {
                                desiredSpeed = current.rampToSpeed;
                            }
                            break;
                        case MILLIS:
                            if(!timeStart){
                                timeUtil.resetTimer();
                                timeStart = true;
                            }
                            desiredSpeed = MathUtil.map(timeUtil.timePassed(),0,current.atDimension, startingSpeed,current.rampToSpeed);
                            nextSpeed = (timeUtil.timePassed()>=current.atDimension);
                            break;
                    }
                    //                        double thisProgress = (d.progress-preProg)/(progressSpeed.atDimension-preProg);
                    //                        desiredSpeed = d.previousSpeed+((progressSpeed.rampToSpeed-d.previousSpeed)*thisProgress);
                    //                        if(desiredSpeed<0.1){
                    //                            desiredSpeed = 0.1;
                    //                        }
                    if(nextSpeed){
                        startingSpeed = desiredSpeed;
                        startingProg = d.progress;
                        currIndex++;
                        timeStart = false;
                        nextSpeed = false;
                    }
                }
                d.previousSpeed = desiredSpeed;
                return desiredSpeed;
            }
        };
    }


//    public static Interfaces.SpeedCalc setAbstractSpeed(ControlSpeed... controlSpeeds){
//
//        return new Interfaces.SpeedCalc(){
//            private double desiredSpeed = 0.1;//set this to minimum moving speed
//            private boolean timeStart = false;
//            private double thisProgress = 0;
//            private int currIndex = -1;
//            private double startingSpeed = 0;
//            private double startingProg = 0;
//            private boolean nextSpeed = false;
//            TimeUtil timeUtil = new TimeUtil();
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//
//            @Override
//            public double CalcSpeed(Interfaces.MoveData d){
//                if(currIndex<0){
//                    timeStart = false;
//                    startingSpeed = d.previousSpeed;
//                    startingProg = 0;
//                    currIndex = 0;
//                }
//
//                if(currIndex < controlSpeeds.length){
//                    ControlSpeed current = controlSpeeds[currIndex];
//                    //                for(int i=0;i<progressSpeed .length;i++){
//
//                    if (current.doProgNotTime == true) {
//                        desiredSpeed = MathUtil.map(d.progress, startingProg, current.atProg, startingSpeed, current.rampToSpeed);
//                        nextSpeed = (d.progress >= current.atProg);
//                    } else if (current.doProgNotTime == false) {
//                        if (!timeStart) {
//                            timeUtil.resetTimer();
//                            timeStart = true;
//                        }
//                        desiredSpeed = MathUtil.map(timeUtil.timePassed(), 0, current.atTime, startingSpeed, current.rampToSpeed);
//                        nextSpeed = (timeUtil.timePassed() >= current.atTime);
//                    }
//
//                    if(nextSpeed){
//                        startingSpeed = desiredSpeed;
//                        startingProg = d.progress;
//                        currIndex++;
//                        timeStart = false;
//                        nextSpeed = false;
//                    }
//                }
//                d.previousSpeed = desiredSpeed;
//                return desiredSpeed;
//            }
//        };
//    }

//    public abstract class ControlSpeed{
//        boolean doProgNotTime = true;
//        double rampToSpeed = 0;
//        double atTime = 0;
//        double atProg = 0;
//
//    }
//    public static ControlSpeed controlSpeed(){
//        public class TimeSpeed extends ControlSpeed{
//            public TimeSpeed(double rampToSpeed, double byTime){
//                this.doProgNotTime = false;
//                this.rampToSpeed = rampToSpeed;
//                this.atTime = byTime;
//            }
//        }
//
//    }
//
//    public class ProgSpeed extends ControlSpeed{
//        public ProgSpeed(double rampToSpeed, double byProg){
//            this.doProgNotTime = true;
//            this.rampToSpeed = rampToSpeed;
//            this.atProg = byProg;
//        }
//    }


    public static class ProgressSpeed{
        public enum timeOrProg{
            MILLIS,
            PROG
        }
        public double rampToSpeed;
        public double atDimension;
        timeOrProg timeProg;
        public ProgressSpeed(double rampToSpeed, double atDimension, timeOrProg timeOrProg){
            this.rampToSpeed = rampToSpeed;
            this.atDimension = atDimension;
            this.timeProg = timeOrProg;
        }
    }

}
