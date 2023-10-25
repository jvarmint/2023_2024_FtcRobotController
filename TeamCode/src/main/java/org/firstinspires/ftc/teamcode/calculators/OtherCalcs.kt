package org.firstinspires.ftc.teamcode.calculators


import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData
import org.firstinspires.ftc.teamcode.calculators.Interfaces.OtherCalc
import org.firstinspires.ftc.teamcode.hardware.robertomap.RobotMap
import org.firstinspires.ftc.teamcode.utilities.TimeUtil
import org.firstinspires.ftc.teamcode.utilities.Vector2D

//import org.firstinspires.ftc.teamcode.Hardware.Sensors.GoalPositionPipeline;
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.PowerShotPositionPipeline;
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.pipelines.StackDeterminationPipeline;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
object OtherCalcs {

    fun OtherLambda(calcLambda: (MoveData) -> Unit, progressLambda: (MoveData) -> Double) : OtherCalc{
        return object: OtherCalc {
            override fun myProgress(d: MoveData): Double {
                return progressLambda(d)
            }

            override fun CalcOther(d: MoveData) {
                calcLambda(d)
            }

        }
    }

    fun OtherLambda(calcLambda: (MoveData) -> Unit) : OtherCalc{
        return object: OtherCalc {
            override fun myProgress(d: MoveData): Double {
                return 0.0
            }

            override fun CalcOther(d: MoveData) {
                calcLambda(d)
            }

        }
    }

    fun TeleAlignPost(): OtherCalc {
        return object : OtherCalc {
            override fun CalcOther(d: MoveData) {
                if (d.manip.y() || d.driver.y()) {
                    val leftCameraDirection = Vector2D(1.0, 0.0)
                    leftCameraDirection.rotateBy(Math.toRadians(-26.0))
                    val rightCameraDirection = Vector2D(1.0, 0.0)
                    rightCameraDirection.rotateBy(Math.toRadians(55.0))
                    val leftError = d.robot.leftPoleAlignPipeline.distanceFromCenterHigh() / 120.0
                    val rightError = d.robot.rightPoleAlignPipeline.distanceFromCenterHigh() / 120.0
                    //                    if(leftError>.5)
//                        leftError=.5;
//                    if(rightError>.5)
//                        rightError=.5;
                    val tempTotalVector = leftCameraDirection.getMultiplied(-leftError)
                        .getAdded(rightCameraDirection.getMultiplied(-rightError))
                    if (tempTotalVector.length > .4) {
                        d.robotCentricAdditiveVector =
                            tempTotalVector.normalized.getDivided(1 / 0.4)
                    } else {
                        d.robotCentricAdditiveVector = tempTotalVector
                    }
                } else {
                    d.robotCentricAdditiveVector = Vector2D()
                }
            }

            override fun myProgress(d: MoveData): Double {
                return 0.0
            }
        }
    }

    @JvmStatic
    fun AutoAlignPost(): OtherCalc {
        return object : OtherCalc {
            override fun CalcOther(d: MoveData) {
                val leftCameraDirection = Vector2D(1.0, 0.0)
                leftCameraDirection.rotateBy(Math.toRadians(-26.0))
                val rightCameraDirection = Vector2D(1.0, 0.0)
                rightCameraDirection.rotateBy(Math.toRadians(55.0))
                val leftError =
                    d.robot.leftPoleAlignPipeline.distanceFromCenterHigh() / 90.0 // was 120
                val rightError =
                    d.robot.rightPoleAlignPipeline.distanceFromCenterHigh() / 30.0 // was 120
                //                    if(leftError>.5)
//                        leftError=.5;
//                    if(rightError>.5)
//                        rightError=.5;
                val tempTotalVector = leftCameraDirection.getMultiplied(-leftError)
                    .getAdded(rightCameraDirection.getMultiplied(-rightError))
                if (tempTotalVector.length > .22) { // was .4
                    d.robotCentricAdditiveVector =
                        tempTotalVector.normalized * .22 // was .4
                } else {
                    d.robotCentricAdditiveVector = tempTotalVector
                }
            }

            override fun myProgress(d: MoveData): Double {
                return 0.0
            }
        }
    }

    fun Raise(): OtherCalc {
        return object : OtherCalc {
            var myProgress = 0.0
            var state = 0
            var startTime = 0.0
            override fun CalcOther(d: MoveData) {
                when (state) {
                    0 -> {
                        RobotMap.lift.targetPosition = 1570 //was 1620
                        RobotMap.liftEx.velocity = 1500.0
                        RobotMap.pitch.position = .9
                        if (RobotMap.lift.currentPosition >= 1500) state++ //was 1600
                    }
                    1 -> {
                        startTime = System.currentTimeMillis().toDouble()
                        state++
                    }
                    2 -> {
                        RobotMap.pitch.position = .46
                        if (System.currentTimeMillis() - startTime > 1300) {
                            startTime = System.currentTimeMillis().toDouble()
                            state++
                        }
                    }
                    3 -> {
                        RobotMap.claw.position = 0.4
                        if (System.currentTimeMillis() - startTime > 500) {
                            startTime = System.currentTimeMillis().toDouble()
                            state++
                        }
                    }
                    else -> myProgress = 1.0
                }
            }

            override fun myProgress(d: MoveData): Double {
                return myProgress
            }
        }
    }

    fun LiftToPos(pos: Int): OtherCalc {
        return object : OtherCalc {
            override fun CalcOther(d: MoveData) {

                RobotMap.lift.targetPosition = pos
                RobotMap.liftEx.velocity = 1500.0
            }

            override fun myProgress(d: MoveData): Double {
                return 0.0
            }
        }
    }

    fun ShakeClaw(): OtherCalc {
        return object : OtherCalc {
            var myProgress = 0.0
            override fun CalcOther(d: MoveData) {
                if (d.progress < 0.5) {
                    RobotMap.pitch.position = .55
                } else {
                    RobotMap.pitch.position = .75
                }
            }

            override fun myProgress(d: MoveData): Double {
                return myProgress
            }
        }
    }

    fun Lower(): OtherCalc {
        return object : OtherCalc {
            var myProgress = 0.0
            override fun CalcOther(d: MoveData) {
                RobotMap.lift.targetPosition = 10
                RobotMap.liftEx.velocity = 1720.0 / 3.0
                RobotMap.pitch.position = .75
                if (RobotMap.lift.currentPosition <= 10) myProgress = 1.0
            }

            override fun myProgress(d: MoveData): Double {
                return myProgress
            }
        }
    }

    @JvmStatic
    fun Lift(): OtherCalc {
        return object : OtherCalc {
            var isLiftUp = false
            var targetLiftPos = 10
            override fun CalcOther(d: MoveData) {

                //bottom position 0.4375
                //slight position 0.5
                //top position 0.75

                if (d.manip.y()) {
                    d.initialLiftPos = RobotMap.lift.currentPosition
                }

                //pitch control
                //left joystick is full up -> pitch should be vertical
                //left joystick is full down -> pitch full horizontal
                //let go of joystick go to original position
                if (d.manip.a()) {
                    RobotMap.claw.position = 0.4
                } else {
                    RobotMap.claw.position = .23
                }
                if (d.manip.u()) {
                    isLiftUp = true
                    targetLiftPos = 1580 //1825->1700->1580;
                } else if (d.manip.d()) {
                    isLiftUp = false
                    targetLiftPos = 10
                } else if (d.manip.lb()) {
                    isLiftUp = true
                    targetLiftPos = 270
                } else if (d.manip.rb()) {
                    isLiftUp = true
                    targetLiftPos = 200
                } else if (d.manip.l()) {
                    isLiftUp = true
                    targetLiftPos = 130
                } else if (d.manip.r()) {
                    isLiftUp = true
                    targetLiftPos = 60
                }
                var defaultPosition = 0.0
                if (isLiftUp) defaultPosition = 0.2
                targetLiftPos += (d.manip.rs().y * 20).toInt()
                if (targetLiftPos > 1580) targetLiftPos = 1580
                if (targetLiftPos < 10) targetLiftPos = 10
                RobotMap.lift.targetPosition = targetLiftPos+d.initialLiftPos
                RobotMap.liftEx.velocity = 1825.0 ///2.0);
                var pitchPercentPosition = 0.0
                pitchPercentPosition = 1.0 - d.manip.rt()
                /*                if(d.manip.ls().y < 0.0) {
                    pitchPercentPosition = (1.0 - defaultPosition) * -d.manip.ls().y + defaultPosition;
                } else if (d.manip.ls().y > 0.0) {
                    //if down
                    pitchPercentPosition = defaultPosition * -d.manip.ls().y + defaultPosition;
                } else {
                    pitchPercentPosition = defaultPosition;
                }*/RobotMap.pitch.position = (0.75 - .4 /*0.4375*/) * pitchPercentPosition + .4
            }

            override fun myProgress(d: MoveData): Double {
                return 0.0
            }
        }
    }

    fun whileOpMode(): OtherCalc {
        return object : OtherCalc {
            var myProgress = 0.0
            override fun myProgress(d: MoveData): Double {
                return myProgress
            }

            override fun CalcOther(d: MoveData) {
                myProgress = 0.5
            }
        }
    }

    fun TimeProgress(millis: Double): OtherCalc {
        return object : OtherCalc {
            val initialMillis = System.currentTimeMillis()
            override fun CalcOther(d: MoveData) {}
            override fun myProgress(d: MoveData): Double {
                return (System.currentTimeMillis() - initialMillis) / millis
            }
        }
    }

    fun PIDTest(): OtherCalc {
        return object : OtherCalc {
            override fun CalcOther(d: MoveData) {
                if (d.manip.u()) {
                }
            }

            override fun myProgress(d: MoveData): Double {
                return 0.0
            }
        }
    }

    fun TeleOpMatch(): OtherCalc {
        val matchTime = TimeUtil()
        val endGameTime = TimeUtil()
        return object : OtherCalc {
            private var myProgress = 0.0
            private var firstLoop = true
            override fun myProgress(d: MoveData): Double {
                return myProgress
            }

            override fun CalcOther(d: MoveData) {
                if (firstLoop) {
                    endGameTime.startTimer(120000)
                    matchTime.startTimer(150000)
                    firstLoop = false
                }
                d.timeRemainingUntilEndgame = endGameTime.timeRemaining()
                d.timeRemainingUntilMatch = matchTime.timeRemaining()
                myProgress = 1 - d.timeRemainingUntilMatch / 150000
            }
        }
    }

    @JvmStatic
    fun TelemetryPosition(): OtherCalc {
        return object : OtherCalc {
            protected var stringFieldWidth = 24
            protected var stringFieldHeight = 16
            override fun CalcOther(d: MoveData) {
                val realFieldWidth = 6
                val realFieldHeight = 6
                val adjustedColumn =
                    Math.round(d.wPos.x / realFieldWidth * stringFieldWidth).toInt()
                val adjustedRow = Math.round(d.wPos.y / realFieldHeight * stringFieldHeight).toInt()
                var rval = ""
                for (row in stringFieldHeight - 1 downTo -1 + 1) {
                    if (row == stringFieldHeight - 1) rval += "\u2004______________________________________________\n"
                    rval += "|"
                    for (col in 0 until stringFieldWidth) {
                        rval += if (row == adjustedRow && col == adjustedColumn) {
                            "■"
                        } else {
                            "\u2004\u2002"
                        }
                        if (col == stringFieldWidth - 1) {
                            rval += if (row == 0) {
                                "|"
                            } else {
                                "|\n"
                            }
                        }
                    }
                }
                rval += "_______________________________________________\u2004"
                d.field = rval
            }

            override fun myProgress(d: MoveData): Double {
                return 0.0
            }
        }
    }

    fun AutoOpMatch(): OtherCalc {
        val autoTime = TimeUtil()
        return object : OtherCalc {
            private var myProgress = 0.0
            protected var timeInAuto = 30000
            private var firstLoop = true
            override fun CalcOther(d: MoveData) {
                if (firstLoop) {
                    autoTime.startTimer(timeInAuto)
                    firstLoop = false
                }
                myProgress = 1 - autoTime.timeRemaining() / timeInAuto
            }

            override fun myProgress(d: MoveData): Double {
                return myProgress
            }
        }
    }

    enum class Controller {
        DRIVER, MANIP
    }

    enum class Side {
        FRONT, BACK, RIGHT, LEFT
    }
}