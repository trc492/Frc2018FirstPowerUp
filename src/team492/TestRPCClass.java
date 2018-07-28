package team492;

import java.util.Date;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class TestRPCClass 
{
	@JsonProperty
	private String firstName;
	private String lastName;
	private long timeInitialized = 0;
	
	@JsonCreator
	public TestRPCClass(@JsonProperty("firstName") String firstName, @JsonProperty("lastName") String lastName)
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
