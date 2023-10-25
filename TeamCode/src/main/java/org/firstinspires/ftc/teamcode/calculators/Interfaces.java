package org.firstinspires.ftc.teamcode.calculators;


//import org.firstinspires.ftc.teamcode.Hardware.Sensors.pipelines.StackDeterminationPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap;
import org.firstinspires.ftc.teamcode.hardware.sensors.CompleteController;
import org.firstinspires.ftc.teamcode.utilities.*;
//import org.opencv.core.Rect;


public class Interfaces {


    public static class MoveData{

        public Telemetry telemetry;

//        public static double startHeading = -111111111.0;
        public static boolean startHeadingIsSet = false;


        public int initialLiftPos;


        /**
         * @MoveData includes the information necessary to talk between the different calculators
         *      and eventually tell the robot through the {@link RobotMap}
         * @see Command is the information on how the robot is going to move: speed, motionSpeed, orientationSpeed
         */
        public static class Command{
            public double speed;
            public Vector2D motionSpeed;
            public double orientationSpeed;
            public Command(double speed, Vector2D motionSpeed, double orientationSpeed){
                this.speed = speed;
                this.motionSpeed = motionSpeed;
                this.orientationSpeed = orientationSpeed;
            }
        }
        public Command currentCommand = null;
        public Command lastCommand = null;


        /**
         * StartPos and StartHeading is defined at the beginning of the game code
         * {@link org.firstinspires.ftc.teamcode.ftc10650.auto.ExampleAuto} for an example of this
         */
        public static class StartData {
            public Vector2D StartPos;
            public double StartNorthOffset;
            public StartData(Vector2D startPos, double startNorthOffset){
                this.StartPos = startPos;
                this.StartNorthOffset = startNorthOffset;
            }
        }
        public StartData startData = null;


        /**
         * {@link RobotMap}  is referenced to the {@link MoveData} via {@link org.firstinspires.ftc.teamcode.op.ComplexOp}
         */
        public RobotMap robot = null;

        public static double gateOpen = 0.6;
        public static double gateClose = 0.375;

//        public static double initBarmPos = 0;
//        public static double initTarmPos = 0;
//        public static double initSarmPos = 0;
//        public static double lastFrameBarmPos = 0;
//        public static double lastFrameTarmPos = 0;
//        public static double lastFrameSarmPos = 0;
//        public static double firstFrameBarmPos = 0;
//        public static double firstFrameTarmPos = 0;
//        public static double firstFrameSarmPos = 0;
        public static int firstLiftPos;
        public static boolean initArmValid = false;

        public final int initBarmPosOffsetFromZeroTicsToHorizontal = -48;
        public final int initTarmPosOffsetFromZeroTicsToHorizontal = -1320;

//        public Arm arm = new Arm(this);

        public float barmAngle = 0;
        public final double tickPerDegreeBarm = (926-434)/45.0; //916 turned 45 downwards 434
        public final double maxTickPerSecondBarm = 0;

        public float tarmAngle = 0;
        public double tickPerDegreeTarm = (92-(-599))/90.0; //start 92 turned 90 upwards -599
        public double maxTickPerDegreeTarm = 0;

        public float sarmAngle = 0;
        public final double tickPerDegreeSarm = 1876/180.0; //start 0 turned 180 to 1876
        public final double maxTickPerDegreeSarm = 0;

        public boolean aimToPowerOverride = false;

        public final int topLiftPos = 1585;
        public final int middleLiftPos = 1100;//800
        public final int bottomLiftPos = 800;//500
        public final int safeLiftPos = 500;
        public final int cameraLiftPos = 500;
        public final int intakeLiftPos = 5;

        public boolean holdPosition = false;

        public final int[] cubeLiftPositions = {
                bottomLiftPos,
                middleLiftPos,
                topLiftPos
        };




        /**
         * the reason the gyro is not passed as a gyro into move data and instead as a heading is so that all
         * code uses the same heading every loop
         *
         * heading has 0 deg as true north and goes positive in a clockwise direction
         */
        public double heading;


        public Vector2D robotCentricAdditiveVector = new Vector2D();


        public double timeRemainingUntilEndgame = 0;//This could be used in tele op programs to have autonomous overrides
        // i.e. a tape measure shooting into the corner by itself
        public double timeRemainingUntilMatch = 0;//Same thing as above
        public CompleteController driver, manip;//This is the initialization of the controllers
        public boolean isStarted, isFinished;//This is used in complex op for the opmode but it can be used elsewhere as information


        /**
         * @see Vector2D wPos is the current robot world position
         *      this is updated outside of the calculators
         * @see Vector2D preWPos is the previous frames world position
         *      it is updated inside of the calculators
         * @see Vector2D encoderPos is updated by the encoder matricies
         *      that determine the x and y translation this is less complex
         *      than wPos because it does not account for orientation
         */
        public Vector2D wPos = new Vector2D();
        public Vector2D preWPos = new Vector2D();//this is updated inside of the calculator
        public Vector2D encoderPos = new Vector2D();
        public Vector2D t265Pos = new Vector2D();
        public long encodePosUpdateTimeMillis = 0;


        /**
         * these are all related to the orientation calc most of this could be only inside of orientation calc
         * but this allows for updating the pid inside of game code or with a controller
         * it allows for interesting opportunities
         */
        public int currentSpin = 0;
        public boolean foundSpin = false;
        public double orientationError = 0;
        public double orientationP = 0.008;//0.02//0.01//0.005
        public double orientationI;
        public double orientationD;


        public double previousSpeed = 0;//allowing for speed to be transitioned well


        /**
         * Progress is what is used to determine how far along the current path the robot is
         * any calculator can set the progress which allows for different objects such as
         * distance sensors or a turn to determine when it finishes
         */
        public double progress;


        /**
         * This is for a field in telemetry we will see how this goes
         */
        public String field = "";


        /**
         * these debug data variables allow information to be passed from a calculator to the telemetry
         * @see org.firstinspires.ftc.teamcode.op.ComplexOp ln. ~70
         */
        public double debugData1 = 0;
        public double debugData2 = 0;
        public boolean debugDataBool;

        static double MAXBUCKET = 1.0;
        static double MINBUCKET = 0.5;

        public int stackHeight = -1;



        public int duckPos = -1;

        public Vector2D powerCenter = new Vector2D(-1, -1);

        public double powerError = Double.MAX_VALUE;

        public Vector2D goalBox = new Vector2D(-1, -1);

        public double[] hsvValues = new double[3];
    }


    /**
     * this is defined by every calc that is ever made, if it does not have the ability to update
     * the progress it returns 0
     */
    public interface ProgressCalc{
        double myProgress(MoveData d);
    }


    /**
     * returns a unit Vector that determines the correct direction of translation without bothering with orientation
     */
    public interface MotionCalc extends ProgressCalc{
        Vector2D CalcMotion(MoveData d);
    }


    /**
     * returns a double that is the turning with a PID (most of the time)
     */
    public interface OrientationCalc extends ProgressCalc{
        double CalcOrientation(MoveData d);
    }


    /**
     * This is controlling the speed for the movement of the robot independently of where the robot is headed
     */
    public interface SpeedCalc extends ProgressCalc{
        double CalcSpeed(MoveData d);
    }


    /**
     * this is where anything that is not above can be programed such as what happens when a button is pressed etc.
     */
    public interface OtherCalc extends ProgressCalc{
        void CalcOther(MoveData d);
    }

}
