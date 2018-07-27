package team492;

import java.util.Date;

public class TestRPCClass 
{
	private String firstName;
	private String lastName;
	private long timeInitialized = 0;
	
	public TestRPCClass(String firstName, String lastName)
	{
		this.firstName = firstName;
		this.lastName = lastName;
	}
	
	public void initTime()
	{
		Date date = new Date();
		timeInitialized = date.getTime();
	}
	
	@Override
	public String toString()
	{
		return firstName + " " + lastName + " " + timeInitialized;
	}
}
