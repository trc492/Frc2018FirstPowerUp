/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Relay.Direction;
import frclib.FrcAHRSGyro;
import frclib.FrcAnalogInput;
import frclib.FrcCANTalon;
import frclib.FrcDigitalOutput;
import frclib.FrcEmic2TextToSpeech;
import frclib.FrcI2cLEDPanel;
import frclib.FrcJoystick;
import frclib.FrcPneumatic;
import frclib.FrcRobotBase;
import frclib.FrcRobotBattery;
import hallib.HalDashboard;
import team492.PixyVision.TargetInfo;
import trclib.TrcAnalogInput;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcEmic2TextToSpeech.Voice;
import trclib.TrcFilter;
import trclib.TrcGyro;
import trclib.TrcGyro.DataType;
import trclib.TrcMaxbotixSonarArray;
import trclib.TrcPidController;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcPidDrive;
import trclib.TrcRobot.RunMode;
import trclib.TrcRobotBattery;
import trclib.TrcSpuriousFilter;
import trclib.TrcUtil;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends FrcRobotBase
{
    public static final String programName = "FirstPowerUp";
    private static final String moduleName = "Robot";

    public static final boolean USE_TRACELOG = true;
    public static final boolean USE_NAV_X = true;
    public static final boolean USE_USB_CAM = true;
    public static final boolean USE_PIXY_SPI = false;
    public static final boolean USE_PIXY_I2C = true;
    public static final boolean USE_TEXT_TO_SPEECH = true;
    public static final boolean USE_MESSAGE_BOARD = false;
    public static final boolean USE_GYRO_ASSIST = true;
    public static final boolean USE_TORQUE_BASED_DRIVING = false;
    public static final boolean USE_MAXBOTIX_SONAR_SENSOR = true;

    private static final boolean DEBUG_POWER_CONSUMPTION = true;
    private static final boolean DEBUG_DRIVE_BASE = false;
    private static final boolean DEBUG_PID_DRIVE = false;
    private static final boolean DEBUG_WINCH = false;
    private static final boolean DEBUG_ELEVATOR = true;
    private static final boolean DEBUG_CUBE_PICKUP = true;
    private static final boolean DEBUG_PIXY = true;

    private static final double DASHBOARD_UPDATE_INTERVAL = 0.1;
    private static final double SPEAK_PERIOD_SECONDS = 20.0; // Speaks once every this # of second.
    private static final double IDLE_PERIOD_SECONDS = 300.0;

    public DriverStation ds = DriverStation.getInstance();
    public HalDashboard dashboard = HalDashboard.getInstance();
    public TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();

    public double targetHeading = 0.0;

    private double nextUpdateTime = TrcUtil.getCurrentTime();

    //
    // Inputs.
    //
    public FrcJoystick leftDriveStick = null;
    public FrcJoystick rightDriveStick = null;
    public FrcJoystick operatorStick = null;

    //
    // Sensors.
    //
    public PowerDistributionPanel pdp = null;
    public TrcRobotBattery battery = null;
    public TrcGyro gyro = null;
    public AnalogInput pressureSensor = null;
    public TrcMaxbotixSonarArray leftSonar = null;
    public TrcMaxbotixSonarArray rightSonar = null;
    public TrcMaxbotixSonarArray frontSonar = null;

    //
    // VisionTarget subsystem.
    //
    public PixyVision pixy = null;

    //
    // Miscellaneous subsystem.
    //
    public FrcI2cLEDPanel messageBoard = null;
    public FrcEmic2TextToSpeech tts = null;
    private double nextTimeToSpeakInSeconds = 0.0;  //0 means disabled, no need to speak;
    public CubeIndicator cubeIndicator = null;

    //
    // DriveBase subsystem.
    //
    public FrcCANTalon leftFrontWheel;
    public FrcCANTalon leftRearWheel;
    public FrcCANTalon rightFrontWheel;
    public FrcCANTalon rightRearWheel;
    public TrcDriveBase driveBase;

    public TrcPidController encoderXPidCtrl;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroTurnPidCtrl;
    public TrcPidDrive pidDrive;

    //public TrcPidController sonarDrivePidCtrl;
    public TrcPidController visionTurnPidCtrl;
    public TrcPidDrive visionPidDrive;
    public TrcPidDrive sonarPidDrive;
    public TrcPidDrive visionPidTurn;

    //
    // Flippers subsystem.
    //
    public FrcPneumatic leftFlipper;
    public FrcPneumatic rightFlipper;

    //
    // Define our subsystems for Auto and TeleOp modes.
    //
    public Relay ringLightsPower;
    public CubePickup cubePickup;
    public Winch winch;
    public Elevator elevator;
    public CmdAutoCubePickup cmdAutoCubePickup;
    public CmdStrafeUntilCube cmdStrafeUntilCube;

    // FMS provided the following info:
    //  - event name
    //  - match type
    //  - match number
    //  - alliance
    //  - location
    //  - replay number???

    public String eventName = "Unknown";
    public MatchType matchType = MatchType.None;
    public int matchNumber = 0;
    public Alliance alliance = Alliance.Red;
    public int location = 1;
    public String gameSpecificMessage = null;

    public double driveTime;
    public double drivePower;
    public double driveDistance;
    public double drivePowerLimit;
    public double turnDegrees;
    public double frontSonarTarget;
    public double visionTurnTarget;
    public double tuneKp;
    public double tuneKi;
    public double tuneKd;
    public double tuneKf;

    /**
     * Constructor.
     */
    public Robot()
    {
        super(programName);
    }   //Robot

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        //
        // Inputs.
        //
        leftDriveStick = new FrcJoystick("leftDriveStick", RobotInfo.JSPORT_LEFT_DRIVESTICK);
        rightDriveStick = new FrcJoystick("rightDriveStick", RobotInfo.JSPORT_RIGHT_DRIVESTICK);
        operatorStick = new FrcJoystick("operatorStick", RobotInfo.JSPORT_OPERATORSTICK);

        //
        // Sensors.
        //
        pdp = new PowerDistributionPanel(RobotInfo.CANID_PDP);
        battery = new FrcRobotBattery(RobotInfo.CANID_PDP);
        if (USE_NAV_X)
        {
            gyro = new FrcAHRSGyro("NavX", SPI.Port.kMXP);
        }
        pressureSensor = new AnalogInput(RobotInfo.AIN_PRESSURE_SENSOR);

        if (USE_MAXBOTIX_SONAR_SENSOR)
        {
            TrcSpuriousFilter leftSonarFilter =
                new TrcSpuriousFilter("LeftSonarFilter", RobotInfo.SONAR_ERROR_THRESHOLD, tracer);
            FrcAnalogInput leftSonarSensor = new FrcAnalogInput(
                "LeftSonarSensor", RobotInfo.AIN_LEFT_SONAR_SENSOR, new TrcFilter[] {leftSonarFilter});
            leftSonarSensor.setScale(RobotInfo.SONAR_MILLIVOLTS_PER_INCH);
            FrcDigitalOutput leftSonarPing = new FrcDigitalOutput("LeftSonarPing", RobotInfo.DIO_LEFT_SONAR_PING);
            leftSonar = new TrcMaxbotixSonarArray(
                "LeftSonar", new TrcAnalogInput[] {leftSonarSensor}, leftSonarPing);

            TrcSpuriousFilter rightSonarFilter =
                new TrcSpuriousFilter("RightSonarFilter", RobotInfo.SONAR_ERROR_THRESHOLD, tracer);
            FrcAnalogInput rightSonarSensor = new FrcAnalogInput(
                "RightSonarSensor", RobotInfo.AIN_RIGHT_SONAR_SENSOR, new TrcFilter[] {rightSonarFilter});
            rightSonarSensor.setScale(RobotInfo.SONAR_MILLIVOLTS_PER_INCH);
            FrcDigitalOutput rightSonarPing = new FrcDigitalOutput("RightSonarPing", RobotInfo.DIO_RIGHT_SONAR_PING);
            rightSonar = new TrcMaxbotixSonarArray(
                "RightSonar", new TrcAnalogInput[] {rightSonarSensor}, rightSonarPing);

            TrcSpuriousFilter frontSonarFilter =
                new TrcSpuriousFilter("FrontSonarFilter", RobotInfo.SONAR_ERROR_THRESHOLD, tracer);
            FrcAnalogInput frontSonarSensor = new FrcAnalogInput(
                "FrontSonarSensor", RobotInfo.AIN_FRONT_SONAR_SENSOR, new TrcFilter[] {frontSonarFilter});
            frontSonarSensor.setScale(RobotInfo.SONAR_MILLIVOLTS_PER_INCH);
            FrcDigitalOutput frontSonarPing = new FrcDigitalOutput("FrontSonarPing", RobotInfo.DIO_FRONT_SONAR_PING);
            frontSonar = new TrcMaxbotixSonarArray(
                "FrontSonar", new TrcAnalogInput[] {frontSonarSensor}, frontSonarPing);
        }

        //
        // VisionTarget subsystem.
        //
        if (USE_USB_CAM)
        {
            UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture("cam0", 0);
            cam0.setResolution(RobotInfo.USBCAM_WIDTH, RobotInfo.USBCAM_HEIGHT);
            cam0.setFPS(RobotInfo.USBCAM_FRAME_RATE);
            cam0.setBrightness(RobotInfo.USBCAM_BRIGHTNESS);
            CameraServer.getInstance().getVideo(cam0);
            CameraServer.getInstance().putVideo("VisionTarget", RobotInfo.USBCAM_WIDTH, RobotInfo.USBCAM_HEIGHT);
        }

        if (USE_PIXY_SPI)
        {
            pixy = new PixyVision(
                "PixyCam", this, RobotInfo.PIXY_POWER_CUBE_SIGNATURE, RobotInfo.PIXY_BRIGHTNESS,
                RobotInfo.PIXY_ORIENTATION, SPI.Port.kMXP);
        }
        else if(USE_PIXY_I2C)
        {
            pixy = new PixyVision(
                "PixyCam", this, RobotInfo.PIXY_POWER_CUBE_SIGNATURE, RobotInfo.PIXY_BRIGHTNESS,
                RobotInfo.PIXY_ORIENTATION, I2C.Port.kMXP, RobotInfo.PIXYCAM_I2C_ADDRESS);
        }

        //
        // Miscellaneous subsystems.
        //
        if (USE_TEXT_TO_SPEECH)
        {
            tts = new FrcEmic2TextToSpeech("TextToSpeech", SerialPort.Port.kMXP, 9600);
            tts.setEnabled(true);
            tts.selectVoice(Voice.FrailFrank);
            tts.setVolume(0.72);
        }

        if (USE_MESSAGE_BOARD)
        {
            messageBoard = new FrcI2cLEDPanel("messageBoard", I2C.Port.kOnboard);
        }
        cubeIndicator = new CubeIndicator();
        cubeIndicator.showNoCube();

        //
        // DriveBase subsystem.
        //
        leftFrontWheel = new FrcCANTalon("LeftFrontWheel", RobotInfo.CANID_LEFTFRONTWHEEL);
        leftRearWheel = new FrcCANTalon("LeftRearWheel", RobotInfo.CANID_LEFTREARWHEEL);
        rightFrontWheel = new FrcCANTalon("RightFrontWheel", RobotInfo.CANID_RIGHTFRONTWHEEL);
        rightRearWheel = new FrcCANTalon("RightRearWheel", RobotInfo.CANID_RIGHTREARWHEEL);

        //
        // Initialize each drive motor controller.
        //
        leftFrontWheel.setInverted(false);
        leftRearWheel.setInverted(false);
        rightFrontWheel.setInverted(true);
        rightRearWheel.setInverted(true);

        leftFrontWheel.motor.overrideLimitSwitchesEnable(false);
        leftRearWheel.motor.overrideLimitSwitchesEnable(false);
        rightFrontWheel.motor.overrideLimitSwitchesEnable(false);
        rightRearWheel.motor.overrideLimitSwitchesEnable(false);

        leftFrontWheel.setPositionSensorInverted(false);
        leftRearWheel.setPositionSensorInverted(false);
        rightFrontWheel.setPositionSensorInverted(false);
        rightRearWheel.setPositionSensorInverted(false);

        leftFrontWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        leftRearWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rightFrontWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rightRearWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);

        //
        // Initialize DriveBase subsystem.
        //
        driveBase = new TrcDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setXPositionScale(RobotInfo.ENCODER_X_INCHES_PER_COUNT);
        driveBase.setYPositionScale(RobotInfo.ENCODER_Y_INCHES_PER_COUNT);

        if (USE_TORQUE_BASED_DRIVING)
        {
            driveBase.setMotorPowerMapper(this::translateMotorPower);
        }

        if(USE_GYRO_ASSIST)
        {
            driveBase.enableGyroAssist(RobotInfo.GYRO_MAX_ROTATION_RATE, RobotInfo.GYRO_ASSIST_KP);
        }

        //
        // Create PID controllers for DriveBase PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
            "encoderXPidCtrl",
            new PidCoefficients(
                RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD, RobotInfo.ENCODER_X_KF),
            RobotInfo.ENCODER_X_TOLERANCE,
            driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl",
            new PidCoefficients(
                RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD, RobotInfo.ENCODER_Y_KF),
            RobotInfo.ENCODER_Y_TOLERANCE,
            driveBase::getYPosition);
        gyroTurnPidCtrl = new TrcPidController(
            "gyroTurnPidCtrl",
            new PidCoefficients(
                RobotInfo.GYRO_TURN_KP, RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD, RobotInfo.GYRO_TURN_KF),
            RobotInfo.GYRO_TURN_TOLERANCE,
            driveBase::getHeading);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);
        pidDrive.setStallTimeout(RobotInfo.DRIVE_STALL_TIMEOUT);
        pidDrive.setMsgTracer(tracer);

//        sonarDrivePidCtrl = new TrcPidController(
//            "sonarDrivePidCtrl",
//            new PidCoefficients(
//                RobotInfo.SONAR_KP, RobotInfo.SONAR_KI, RobotInfo.SONAR_KD, RobotInfo.SONAR_KF),
//            RobotInfo.SONAR_TOLERANCE,
//            this::getFrontSonarDistance);
//        sonarDrivePidCtrl.setAbsoluteSetPoint(true);
//        sonarDrivePidCtrl.setInverted(true);
        visionTurnPidCtrl = new TrcPidController(
            "visionTurnPidCtrl",
            new PidCoefficients(
                RobotInfo.VISION_TURN_KP, RobotInfo.VISION_TURN_KI, RobotInfo.VISION_TURN_KD, RobotInfo.VISION_TURN_KF),
            RobotInfo.VISION_TURN_TOLERANCE,
            this::getPixyTargetAngle);
        visionTurnPidCtrl.setInverted(true);
        visionTurnPidCtrl.setAbsoluteSetPoint(true);
        visionPidDrive = new TrcPidDrive("visionPidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, visionTurnPidCtrl);
        visionPidDrive.setStallTimeout(RobotInfo.DRIVE_STALL_TIMEOUT);
        visionPidDrive.setMsgTracer(tracer);
//        sonarPidDrive = new TrcPidDrive("sonarPidDrive", driveBase, null, sonarDrivePidCtrl, null);
//        sonarPidDrive.setMsgTracer(tracer);
        visionPidTurn = new TrcPidDrive("visionPidTurn", driveBase, null, null, visionTurnPidCtrl);
        visionPidTurn.setMsgTracer(tracer);

        //
        // Create other hardware subsystems.
        //
        ringLightsPower = new Relay(RobotInfo.RELAY_RINGLIGHT_POWER);
        ringLightsPower.setDirection(Direction.kForward);
        cubePickup = new CubePickup(this);
        winch = new Winch();
        elevator = new Elevator();
        leftFlipper = new FrcPneumatic("leftFlipper", RobotInfo.CANID_PCM1, 
            RobotInfo.SOL_LEFT_FLIPPER_EXTEND, RobotInfo.SOL_LEFT_FLIPPER_RETRACT);
        rightFlipper =  new FrcPneumatic("rightFlipper", RobotInfo.CANID_PCM1, 
            RobotInfo.SOL_RIGHT_FLIPPER_EXTEND, RobotInfo.SOL_RIGHT_FLIPPER_RETRACT);
        cmdAutoCubePickup = new CmdAutoCubePickup(this);
        cmdStrafeUntilCube = new CmdStrafeUntilCube(this);

        //
        // Robot Modes.
        //
        setupRobotModes(new FrcTeleOp(this), new FrcAuto(this), new FrcTest(this), new FrcDisabled(this));
    }   //robotInit

    public void robotStartMode(RunMode runMode)
    {
        if (tts != null)
        {
            if (runMode == RunMode.DISABLED_MODE)
            {
                // Robot is safe.
                // Note: "disaibled" is not a typo. It forces the speech board to pronounce it correctly.
                tts.speak("Robot disaibled");
                nextTimeToSpeakInSeconds = TrcUtil.getCurrentTime() + IDLE_PERIOD_SECONDS;
            }
            else
            {
                // Robot is unsafe
                tts.speak("Robot enabled, stand clear");
                nextTimeToSpeakInSeconds = TrcUtil.getCurrentTime() + SPEAK_PERIOD_SECONDS;
            }
        }

        if (ds.isFMSAttached())
        {
            eventName = ds.getEventName();
            matchType = ds.getMatchType();
            matchNumber = ds.getMatchNumber();
        }
        alliance = ds.getAlliance();
        location = ds.getLocation();
        gameSpecificMessage = ds.getGameSpecificMessage();

        battery.setTaskEnabled(true);
        driveTime = HalDashboard.getNumber("DriveTime", 5.0);
        drivePower = HalDashboard.getNumber("DrivePower", 0.2);
        driveDistance = HalDashboard.getNumber("DriveDistance", 6.0);
        drivePowerLimit = HalDashboard.getNumber("DrivePowerLimit", 0.5);
        turnDegrees = HalDashboard.getNumber("TurnDegrees", 90.0);
        frontSonarTarget = HalDashboard.getNumber("FrontSonarTarget", 7.0);
        visionTurnTarget = HalDashboard.getNumber("VisionTurnTarget", 0.0);
        tuneKp = HalDashboard.getNumber("TuneKp", RobotInfo.GYRO_TURN_KP);
        tuneKi = HalDashboard.getNumber("TuneKi", RobotInfo.GYRO_TURN_KI);
        tuneKd = HalDashboard.getNumber("TuneKd", RobotInfo.GYRO_TURN_KD);
        tuneKf = HalDashboard.getNumber("TuneKf", 0.0);
    }   //robotStartMode

    public void robotStopMode(RunMode runMode)
    {
        driveBase.stop();
        battery.setTaskEnabled(false);
    }   //robotStopMode

    public void setVisionEnabled(boolean enabled)
    {
        if (pixy != null)
        {
            pixy.setEnabled(enabled);
            tracer.traceInfo("Vision", "Pixy is %s!", enabled? "enabled": "disabled");
        }
    }   //setVisionEnabled

    public void updateDashboard()
    {
        double currTime = TrcUtil.getCurrentTime();

        if (currTime >= nextUpdateTime)
        {
            nextUpdateTime = currTime + DASHBOARD_UPDATE_INTERVAL;
            
            HalDashboard.putNumber("gyroTurnRate", gyro.getRawZData(DataType.ROTATION_RATE).value);

            if (DEBUG_POWER_CONSUMPTION)
            {
                HalDashboard.putNumber("pdpTotalCurrent", pdp.getTotalCurrent());
                HalDashboard.putNumber("elevatorCurrent", elevator.elevatorMotor.motor.getOutputCurrent());
                HalDashboard.putNumber("winchCurrent", winch.getCurrent());
                HalDashboard.putNumber("grabberCurrent", cubePickup.getGrabberCurrent());
            }

            if (DEBUG_DRIVE_BASE)
            {
                //
                // DriveBase debug info.
                //
                double lfEnc = leftFrontWheel.getPosition();
                double rfEnc = rightFrontWheel.getPosition();
                double lrEnc = leftRearWheel.getPosition();
                double rrEnc = rightRearWheel.getPosition();

                dashboard.displayPrintf(8, "DriveBase: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f, avg=%.0f",
                    lfEnc, rfEnc, lrEnc, rrEnc, (lfEnc + rfEnc + lrEnc + rrEnc)/4.0);
                dashboard.displayPrintf(9, "DriveBase: X=%.1f, Y=%.1f, Heading=%.1f",
                    driveBase.getXPosition(), driveBase.getYPosition(), driveBase.getHeading());

                if (DEBUG_PID_DRIVE)
                {
                    encoderXPidCtrl.displayPidInfo(10);
                    encoderYPidCtrl.displayPidInfo(12);
                    gyroTurnPidCtrl.displayPidInfo(14);
                }
                HalDashboard.putNumber("DriveBase.X", driveBase.getXPosition());
                HalDashboard.putNumber("DriveBase.Y", driveBase.getYPosition());
                HalDashboard.putNumber("DriveBase.Heading", driveBase.getHeading());
            }

            if (DEBUG_WINCH)
            {
                dashboard.displayPrintf(8, "Winch: power=%.1f", winch.getPower());
            }

            if (DEBUG_ELEVATOR)
            {
                dashboard.displayPrintf(8, "Elevator: power=%.1f, position=%.1f(%.1f), limitSw=%b/%b",
                    elevator.getPower(), elevator.getPosition(), elevator.elevatorMotor.getPosition(),
                    elevator.elevatorMotor.isLowerLimitSwitchActive(),
                    elevator.elevatorMotor.isUpperLimitSwitchActive());
            }

            if (DEBUG_CUBE_PICKUP)
            {
                dashboard.displayPrintf(9, "CubePickup: power=%.1f, current=%.1f, cubeDetected=%b",
                    cubePickup.getPower(), cubePickup.getGrabberCurrent(), cubePickup.cubeDetected());
            }

            if (DEBUG_PIXY)
            {
                if (pixy != null && pixy.isEnabled())
                {
                    PixyVision.TargetInfo targetInfo = pixy.getTargetInfo();
                    if (targetInfo == null)
                    {
                        dashboard.displayPrintf(10, "Pixy: Target not found!");
                    }
                    else
                    {
                        dashboard.displayPrintf(10, "Pixy: x=%d, y=%d, width=%d, height=%d",
                            targetInfo.rect.x, targetInfo.rect.y, targetInfo.rect.width, targetInfo.rect.height);
                        dashboard.displayPrintf(11, "xDistance=%.1f, yDistance=%.1f, angle=%.1f, ultrasonic=%.1f",
                            targetInfo.xDistance, targetInfo.yDistance, targetInfo.angle, getFrontSonarDistance());
                    }
                }
            }
        }
    }   //updateDashboard

    public void announceSafety()
    {
        double currTime = TrcUtil.getCurrentTime();

        if (tts != null && nextTimeToSpeakInSeconds > 0.0 && currTime >= nextTimeToSpeakInSeconds)
        {
            tts.speak("Stand clear");
            nextTimeToSpeakInSeconds = currTime + SPEAK_PERIOD_SECONDS;
        }
    }   //announceSafety

    public void announceIdling()
    {
        double currTime = TrcUtil.getCurrentTime();

        if (tts != null && nextTimeToSpeakInSeconds > 0.0 && currTime >= nextTimeToSpeakInSeconds)
        {
            tts.speak("Robot is idle, please turn off.");
            nextTimeToSpeakInSeconds = currTime + SPEAK_PERIOD_SECONDS;
        }
    }   //announceIdling

    public void startTraceLog(String prefix)
    {
        String filePrefix = prefix != null? prefix: eventName + "_" + matchType.toString();
        if (prefix == null) filePrefix += String.format("%03d", matchNumber);
        tracer.openTraceLog("/home/lvuser/tracelog", filePrefix);
    }   //startTraceLog

    public void stopTraceLog()
    {
        tracer.closeTraceLog();
    }   //stopTraceLog

    public void traceStateInfo(double elapsedTime, String stateName)
    {
        tracer.traceInfo(moduleName, "[%5.3f] %10s: xPos=%6.2f,yPos=%6.2f,heading=%6.1f/%6.1f,volts=%.1f(%.1f)",
            elapsedTime, stateName, driveBase.getXPosition(), driveBase.getYPosition(), driveBase.getHeading(),
            targetHeading, battery.getVoltage(), battery.getLowestVoltage());
    }   //traceStateInfo

    //
    // Getters for sensor values.
    //

    public double getPressure()
    {
        return 50.0*pressureSensor.getVoltage() - 25.0;
    }   //getPressure

    public double getLeftSonarDistance()
    {
        double value = 0.0;

        if (leftSonar != null)
        {
            value = leftSonar.getDistance(0).value;
        }

        return value;
    }   //getLeftSonarDistance

    public double getRightSonarDistance()
    {
        double value = 0.0;

        if (rightSonar != null)
        {
            value = rightSonar.getDistance(0).value;
        }

        return value;
    }   //getRightSonarDistance

    public double getFrontSonarDistance()
    {
        double value = 0.0;

        if (frontSonar != null)
        {
            value = frontSonar.getDistance(0).value;
        }

        return value;
    }   //getFrontSonarDistance

    public double getPixyTargetAngle()
    {
        TargetInfo targetInfo = pixy.getTargetInfo();
        return targetInfo != null? targetInfo.angle: 0.0;
    }

    public double translateMotorPower(double desiredForcePercentage, double ticksPerSecond)
    {
        double rpmAtWheel = (Math.abs(ticksPerSecond) / RobotInfo.DRIVE_ENCODER_COUNTS_PER_ROTATION) * 60.0;
        double rpmAtMotor = rpmAtWheel * RobotInfo.DRIVE_MOTOR_ROTATIONS_PER_WHEEL_ROTATION;
        double constrainedForcePercentage = constrainForcePercentageByElevatorHeight(desiredForcePercentage);
        double constrainedForceOz = constrainedForcePercentage * RobotInfo.MAX_WHEEL_FORCE_OZ;
        double max_torque_at_rpm_in_ounce_inch = Math.max((-0.06467043 * rpmAtMotor) + 343.4, 0.000001);
        double desiredTorqueAtWheelOzIn = constrainedForceOz * RobotInfo.DRIVE_WHEEL_RADIUS_IN;
        double desiredTorqueAtMotorOzIn = desiredTorqueAtWheelOzIn/RobotInfo.DRIVE_MOTOR_ROTATIONS_PER_WHEEL_ROTATION;
        double returnValue = TrcUtil.clipRange(desiredTorqueAtMotorOzIn/max_torque_at_rpm_in_ounce_inch, -1.0, 1.0);
        tracer.traceInfo("TranslateMotorPower", "desiredForcePercentage=%.2f, ticksPerSecond=%.2f, "
            + "rpmAtWheel=%.2f, rpmAtMotor=%.2f, constrainedForcePercentage=%.2f, constrainedForceOz=%.2f, "
            + "maxTorqueAtRPMOzIn=%.2f, desireTorqueAtWheelOzIn=%.2f, desiredTorqueAtMotorOzIn=%.2f, "
            + "returnValue=%.2f",
            desiredForcePercentage, ticksPerSecond, rpmAtWheel, rpmAtMotor, 
            constrainedForcePercentage, constrainedForceOz, max_torque_at_rpm_in_ounce_inch,
            desiredTorqueAtWheelOzIn, desiredTorqueAtMotorOzIn, returnValue);
        return returnValue;
    }

    public double constrainForcePercentageByElevatorHeight(double desiredForcePercentage)
    {
        double heightPercentage = (elevator.getPosition()-RobotInfo.ELEVATOR_POSITION_OFFSET)
            /(RobotInfo.ELEVATOR_MAX_HEIGHT-RobotInfo.ELEVATOR_POSITION_OFFSET);
        double powerPercentage = 1.0 - (heightPercentage*0.85);
        return TrcUtil.clipRange(desiredForcePercentage, -powerPercentage, powerPercentage);
    }

}   //class Robot
