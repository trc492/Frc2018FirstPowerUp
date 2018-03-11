package team492;

import trclib.TrcAnalogSensor;
import trclib.TrcAnalogTrigger;
import trclib.TrcEvent;
import trclib.TrcStateMachine;
import trclib.TrcTimer;
import trclib.TrcRobot;

public class CmdScaleAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdScaleAuto";
    
    private static final double[] distances = new double[] { RobotInfo.NO_SWITCH_DISTANCE, RobotInfo.SWITCH_DISTANCE };
    
    private enum State
    {
        START,
        START_CUBE_PICKUP,
        CUBE_PICKUP,
        DRIVE_TO_SWITCH,
        DRIVE_TO_LANE_3,
        TURN_TO_OPPOSITE_SCALE,
        DRIVE_TO_OPPOSITE_SCALE,
        TURN_TO_SCALE,
        FINISH_DRIVE_TO_SCALE,
        DRIVE_TO_SCALE,
        TURN_TO_FACE_SCALE,
        RAISE_ELEVATOR,
        THROW_CUBE,
        LOWER_ELEVATOR,
        DONE
    }
    
    private enum Position
    {
        LEFT, MIDDLE, RIGHT
    }
    
    private TrcEvent sonarEvent, event;
    private TrcStateMachine<State> sm;
    private Robot robot;
    private double delay;
    private Position startPosition;
    private boolean startRight;
    private boolean scaleRight;
    private TrcAnalogSensor sonarSensor;
    private TrcAnalogTrigger<TrcAnalogSensor.DataType> sonarTrigger;
    private TrcTimer timer;
    public CmdScaleAuto(Robot robot, double delay, double startPosition)
    {
        this.robot = robot;
        
        event = new TrcEvent(moduleName);
        sonarEvent = new TrcEvent(moduleName + ".sonarEvent");
        sm = new TrcStateMachine<>(moduleName);
        
        timer = new TrcTimer(moduleName);
        
        this.delay = delay;
        
        if(startPosition == RobotInfo.LEFT_START_POS) this.startPosition = Position.LEFT;
        else if(startPosition == RobotInfo.RIGHT_START_POS) this.startPosition = Position.RIGHT;
        else this.startPosition = Position.MIDDLE;
        
        startRight = (startPosition == RobotInfo.LEFT_START_POS);
        
        String gameMessage = robot.ds.getGameSpecificMessage();
        scaleRight = (gameMessage.charAt(1) == 'R');
        
        sonarSensor = new TrcAnalogSensor("SonarSensor", 
            this.startPosition == Position.LEFT?robot::getRightSonarDistance : robot::getLeftSonarDistance);
        sonarTrigger = new TrcAnalogTrigger<TrcAnalogSensor.DataType>(
            "SwitchDistanceTrigger", sonarSensor, 0, TrcAnalogSensor.DataType.RAW_DATA, distances,
            this::sonarTriggerEvent);
    }
    
    public void setSonarTriggerEnabled(boolean enabled)
    {
        sonarTrigger.setTaskEnabled(enabled);
        if(enabled) sonarEvent.clear();
    }
    
    private void sonarTriggerEvent(int currZone, int prevZone, double zoneValue)
    {
        if(currZone == 1 && prevZone == 0)
        {
            sonarEvent.set(true);
        }
    }
    
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        if(done || startPosition == Position.MIDDLE) return true;
        
        State state = sm.checkReadyAndGetState();
        
        double distanceFromWall = 0;
        double sonarDistance = 0;
        double xDistance = 0;
        double yDistance = 0;
        
        if(state != null)
        {
            switch(state)
            {
                case START:
                    if(delay != 0.0)
                    {
                        sm.setState(State.START_CUBE_PICKUP);                        
                    } else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.START_CUBE_PICKUP);
                    }
                    break;
                
                case START_CUBE_PICKUP:
                    robot.cmdAutoCubePickup.start();
                    break;
                
                case CUBE_PICKUP:
                    if(robot.cmdAutoCubePickup.cmdPeriodic(elapsedTime))
                    {
                    	robot.elevator.setPosition(RobotInfo.ELEVATOR_OFF_GROUND);
                        sm.setState(State.DRIVE_TO_SWITCH);
                    }
                    break;
                    
                case DRIVE_TO_SWITCH:
                    robot.pidDrive.driveMaintainHeading(0.0, RobotInfo.MOVE_TO_SWITCH_Y_POWER, robot.targetHeading);
                    setSonarTriggerEnabled(true);
                    State nextState;
                    if(scaleRight && startRight || !scaleRight && !startRight)
                    {
                        nextState = State.DRIVE_TO_SCALE;
                    } else
                    {
                        nextState = State.DRIVE_TO_LANE_3;
                    }
                    sm.waitForSingleEvent(sonarEvent, nextState); // TODO: Add a timeout to this                        
                    break;
                
                case DRIVE_TO_LANE_3:
                    sonarDistance = startRight?robot.getLeftSonarDistance():robot.getRightSonarDistance();
                    distanceFromWall = RobotInfo.SWITCH_TO_WALL_DISTANCE - sonarDistance;
                    robot.pidDrive.setTarget(0.0, RobotInfo.SWITCH_TO_LANE_3_DISTANCE, 
                        robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_OPPOSITE_SCALE);
                    break;
                
                case TURN_TO_OPPOSITE_SCALE:
                    robot.targetHeading = startRight?RobotInfo.DRIVE_HEADING_WEST:RobotInfo.DRIVE_HEADING_EAST;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_OPPOSITE_SCALE);
                    break;
                    
                case DRIVE_TO_OPPOSITE_SCALE:
                    yDistance = RobotInfo.FIELD_WIDTH - 2.0*distanceFromWall + RobotInfo.ROBOT_LENGTH;
                    robot.pidDrive.setTarget(0.0, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_SCALE);
                    break;
                 
                case TURN_TO_SCALE:
                    robot.targetHeading = RobotInfo.DRIVE_HEADING_NORTH;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.FINISH_DRIVE_TO_SCALE);
                    break;
                
                case FINISH_DRIVE_TO_SCALE:
                    robot.pidDrive.setTarget(0.0, RobotInfo.LANE_3_TO_SCALE_DISTANCE, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_FACE_SCALE);
                    break;
                    
                case DRIVE_TO_SCALE:
                    setSonarTriggerEnabled(false);
                    robot.pidDrive.cancel();
                    sonarDistance = startRight?robot.getLeftSonarDistance():robot.getRightSonarDistance();
                    distanceFromWall = RobotInfo.SWITCH_TO_WALL_DISTANCE - sonarDistance;
                    xDistance = distanceFromWall - RobotInfo.SCALE_TO_WALL_DISTANCE;
                    if(!startRight) xDistance *= -1;
                    yDistance = RobotInfo.SWITCH_TO_SCALE_DISTANCE;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_FACE_SCALE);
                    break;
                    
                case TURN_TO_FACE_SCALE:
                    robot.targetHeading = startRight?RobotInfo.DRIVE_HEADING_WEST:RobotInfo.DRIVE_HEADING_EAST;
                    robot.pidDrive.setTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                    break;
                    
                case RAISE_ELEVATOR:
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_SCALE_HIGH, event, 0.0);
                    sm.waitForSingleEvent(event, State.THROW_CUBE);
                    break;
                    
                case THROW_CUBE:
                    robot.cubePickup.dropCube(RobotInfo.CUBE_PICKUP_DROP_POWER);
                    timer.set(RobotInfo.DROP_CUBE_TIMEOUT, event);
                    sm.waitForSingleEvent(event, State.LOWER_ELEVATOR);
                    break;
                    
                case LOWER_ELEVATOR:
                    robot.elevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT, event, 0.0);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;
                    
                case DONE:
                    done = true;
                    setSonarTriggerEnabled(false);
                    sm.stop();
                    break;
            }
        }
        return done;
    }
}
