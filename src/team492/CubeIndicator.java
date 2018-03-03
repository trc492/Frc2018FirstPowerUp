package team492;

import edu.wpi.first.wpilibj.Spark;

public class CubeIndicator
{
    public int PORT_NUMBER;
    public Spark light_controller;
    
    public CubeIndicator(int PORT_NUMBER)
    {
        this.PORT_NUMBER = PORT_NUMBER;
        light_controller = new Spark(PORT_NUMBER);
    }
    
    public void showNoCube() 
    {
        // turned off
        light_controller.set(0.0);
    }
    
    public void showCubeFullyGrabbed() 
    {
        // solid green
        light_controller.set(0.73);
    }
    
    public void showCubeInFront() 
    {
        // solid orange
        light_controller.set(0.65);
    }
}
