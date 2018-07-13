package team492;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import hallib.HalDashboard;
import trclib.TrcRobot.RobotCommand;

public class TalonPidTuner implements RobotCommand
{
	private TalonSRX lfMotor, rfMotor, lrMotor, rrMotor;
	private double distance;
	public TalonPidTuner(Robot robot) 
	{
		this.lfMotor = robot.leftFrontWheel.motor;
		this.rfMotor = robot.rightFrontWheel.motor;
		this.lrMotor = robot.leftRearWheel.motor;
		this.rrMotor = robot.rightRearWheel.motor;
		
		refreshData("Test/TuneKp",0.0);
		refreshData("Test/TuneKi",0.0);
		refreshData("Test/TuneKd",0.0);
		refreshData("Test/DriveDistance",0.0);
		refreshData("Test/LFPIDError",0.0);
		refreshData("Test/RFPIDError",0.0);
	}
	
	private void refreshData(String name, double defaultValue)
	{
		HalDashboard.putNumber(name, HalDashboard.getNumber(name, defaultValue));		
	}
	
	public void start()
	{
		lfMotor.setInverted(false);
		lrMotor.setInverted(false);
		rfMotor.setInverted(true);
		rrMotor.setInverted(true);
		
		lrMotor.set(ControlMode.Follower, RobotInfo.CANID_LEFTFRONTWHEEL);
		rrMotor.set(ControlMode.Follower, RobotInfo.CANID_RIGHTFRONTWHEEL);
		
		double kP = HalDashboard.getNumber("Test/TuneKp", 0.0);
		double kI = HalDashboard.getNumber("Test/TuneKi", 0.0);
		double kD = HalDashboard.getNumber("Test/TuneKd", 0.0);
		
		configMotor(lfMotor, 0.5, kP, kI, kD);
		configMotor(rfMotor, 0.5, kP, kI, kD);
		
		this.distance = HalDashboard.getNumber("Test/DriveDistance", 0.0);
		double encoderTicks = distance / RobotInfo.ENCODER_Y_INCHES_PER_COUNT;
		
		lfMotor.set(ControlMode.Position, encoderTicks);
		rfMotor.set(ControlMode.Position, encoderTicks);
	}
	
	private void configMotor(TalonSRX motor, double maxOutput,  double kP, double kI, double kD)
	{
		motor.config_kP(0, kP, 0);
		motor.config_kI(0, kI, 0);
		motor.config_kD(0, kD, 0);
		motor.config_kF(0, 0.0, 0);
		
		motor.setSelectedSensorPosition(0, 0, 0);
		while (motor.getSelectedSensorPosition(0) != 0)
        {
            Thread.yield();
        }
		
		motor.configClosedLoopPeakOutput(0, maxOutput, 0);
	}

	@Override
	public boolean cmdPeriodic(double elapsedTime)
	{
		double lfPos = lfMotor.getSelectedSensorPosition(0) * RobotInfo.ENCODER_Y_INCHES_PER_COUNT;
		double rfPos = rfMotor.getSelectedSensorPosition(0) * RobotInfo.ENCODER_Y_INCHES_PER_COUNT;
		double lfError = distance - lfPos;
		double rfError = distance - rfPos;
		
		HalDashboard.getInstance().displayPrintf(8, "Time=%.2f : Pos=%.1f,Target=%.1f,Err=%.1f",
				elapsedTime, lfPos, distance, lfError);
		HalDashboard.getInstance().displayPrintf(9, "Time=%.2f : Pos=%.1f,Target=%.1f,Err=%.1f",
				elapsedTime, rfPos, distance, rfError);
		
		HalDashboard.putNumber("Test/LFPIDError", lfError);
		HalDashboard.putNumber("Test/RFPIDError", rfError);
		return false;
	}

}
