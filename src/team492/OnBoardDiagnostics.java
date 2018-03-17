package team492;

public class OnBoardDiagnostics {
	
	private Robot robot;
	
	public OnBoardDiagnostics(Robot robot)
	{
		this.robot = robot;
	}
	
	public void printDiagnostics()
	{
		boolean faults = false;
		if(checkLeftFrontEncoder())
		{
			faults = true;
			System.out.println("left front encoder might be unplugged");
		}
		if(checkRightFrontEncoder())
		{
			faults = true;
			System.out.println("right front encoder might be unplugged");
		}
		if(checkLeftRearEncoder()) 
		{
			faults = true;
			System.out.println("left rear encoder might be unplugged");
		}
		if(checkRightRearEncoder())
		{
			faults = true;
			System.out.println("right rear encoder might be unplugged");
		}
		if(!faults)
		{
		    System.out.println("No faults");
		}
	}
	
	private boolean checkLeftFrontEncoder()
	{
		return robot.leftFrontWheel.getPosition() == 0.0;
	}
	
	private boolean checkRightFrontEncoder()
	{
		return robot.rightFrontWheel.getPosition() == 0.0;
	}
	
	private boolean checkLeftRearEncoder()
	{
		return robot.leftRearWheel.getPosition() == 0.0;
	}
	
	private boolean checkRightRearEncoder()
	{
		return robot.rightRearWheel.getPosition() == 0.0;
	}
}
