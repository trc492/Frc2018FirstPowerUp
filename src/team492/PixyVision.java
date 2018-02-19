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

import java.util.ArrayList;

import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import frclib.FrcPixyCam;
import trclib.TrcPixyCam.ObjectBlock;

public class PixyVision
{
    private static final String moduleName = "PixyVision";
    private static final boolean debugEnabled = true;

    private static final boolean FILTER_ENABLED = false;
    private static final double PERCENT_TOLERANCE = 0.3;    // 30% tolerance
    private static final double PERCENT_TOLERANCE_LOWER = 1.0 - PERCENT_TOLERANCE;
    private static final double PERCENT_TOLERANCE_UPPER = 1.0 + PERCENT_TOLERANCE;

    public class TargetInfo
    {
        public Rect rect;
        public double xDistance;
        public double yDistance;
        public double angle;

        public TargetInfo(Rect rect, double xDistance, double yDistance, double angle)
        {
            this.rect = rect;
            this.xDistance = xDistance;
            this.yDistance = yDistance;
            this.angle = angle;
        }   //TargetInfo

        public String toString()
        {
            return String.format("Rect[%d,%d,%d,%d], xDistance=%.1f, yDistance=%.1f, angle=%.1f",
                rect.x, rect.y, rect.width, rect.height, xDistance, yDistance, angle);
        }
    }   //class TargetInfo

    public enum Orientation
    {
        NORMAL_LANDSCAPE,
        CLOCKWISE_PORTRAIT,
        ANTICLOCKWISE_PORTRAIT,
        UPSIDEDOWN_LANDSCAPE
    }   //enum Orientation

    private static final double PIXY_DISTANCE_SCALE = 2300.0;   //DistanceInInches*targetWidthdInPixels
//    private static final double TAPE_WIDTH_INCHES = 2.0;
    private static final double POWER_CUBE_HEIGHT_INCHES = 11.0; // 11 in tall
    private static final double TARGET_WIDTH_INCHES = 13.0 * Math.sqrt(2); // 13x13 square, diagonal is 13*sqrt(2) inches
    private static final double TARGET_HEIGHT_INCHES = POWER_CUBE_HEIGHT_INCHES;

    private FrcPixyCam pixyCamera;
    private Robot robot;
    private int signature;
    private Orientation orientation;
//    private FrcPneumatic targetFoundLED = null;
//    private FrcPneumatic targetAlignedLED = null;

    private void commonInit(Robot robot, int signature, int brightness, Orientation orientation)
    {
        this.robot = robot;
        this.signature = signature;
        this.orientation = orientation;
        pixyCamera.setBrightness((byte)brightness);
//        targetFoundLED = new FrcPneumatic("TargetFoundLED", RobotInfo.CANID_PCM1, RobotInfo.SOL_TARGET_FOUND_LED);
//        targetAlignedLED = new FrcPneumatic("TargetAlignedLED", RobotInfo.CANID_PCM1, RobotInfo.SOL_TARGET_ALIGNED_LED);
    }   //commonInit

    public PixyVision(
        final String instanceName, Robot robot, int signature, int brightness, Orientation orientation,
        SPI.Port port)
    {
        pixyCamera = new FrcPixyCam(instanceName, port);
        commonInit(robot, signature, brightness, orientation);
    }   //PixyVision

    public PixyVision(
        final String instanceName, Robot robot, int signature, int brightness, Orientation orientation,
        I2C.Port port, int i2cAddress)
    {
        pixyCamera = new FrcPixyCam(instanceName, port, i2cAddress);
        commonInit(robot, signature, brightness, orientation);
    }   //PixyVision

    public void setEnabled(boolean enabled)
    {
        pixyCamera.setEnabled(enabled);
    }   //setEnabled

    public boolean isEnabled()
    {
        return pixyCamera.isEnabled();
    }   //isEnabled

    /**
     * This method analyzes all the detected object rectangles and attempts to find a pair that are the likely targets.
     * It then returns the rectangle enclosing the two object rectangles.
     *
     * @return rectangle of the detected target.
     */
    private Rect getTargetRect()
    {
        //CodeReview: this code is for last year. Needs updating...
        Rect targetRect = null;
        ObjectBlock[] detectedObjects = pixyCamera.getDetectedObjects();

        if (debugEnabled)
        {
            robot.tracer.traceInfo(moduleName, "%s object(s) found",
                detectedObjects != null? "" + detectedObjects.length: "null");
        }
        //
        // Make sure the camera detected at least one objects.
        //
        if (detectedObjects != null && detectedObjects.length >= 1)
        {
            ArrayList<Rect> objectList = new ArrayList<>();
            double targetDistance = robot.getFrontSonarDistance() + RobotInfo.PIXY_CAM_OFFSET;
            //
            // Filter out objects that don't have the correct signature.
            //
            for (int i = 0; i < detectedObjects.length; i++)
            {
                if (signature == detectedObjects[i].signature)
                {
                    int temp;
                    //
                    // If we have the camera mounted in other orientations, we need to adjust the object rectangles
                    // accordingly.
                    //
                    switch (orientation)
                    {
                        case CLOCKWISE_PORTRAIT:
                            temp = RobotInfo.PIXYCAM_WIDTH - detectedObjects[i].centerX;
                            detectedObjects[i].centerX = detectedObjects[i].centerY;
                            detectedObjects[i].centerY = temp;
                            temp = detectedObjects[i].width;
                            detectedObjects[i].width = detectedObjects[i].height;
                            detectedObjects[i].height = temp;
                            break;

                        case ANTICLOCKWISE_PORTRAIT:
                            temp = detectedObjects[i].centerX;
                            detectedObjects[i].centerX = RobotInfo.PIXYCAM_HEIGHT - detectedObjects[i].centerY;
                            detectedObjects[i].centerY = temp;
                            temp = detectedObjects[i].width;
                            detectedObjects[i].width = detectedObjects[i].height;
                            detectedObjects[i].height = temp;
                            break;

                        case UPSIDEDOWN_LANDSCAPE:
                            detectedObjects[i].centerX = RobotInfo.PIXYCAM_WIDTH - detectedObjects[i].centerX;
                            detectedObjects[i].centerY = RobotInfo.PIXYCAM_HEIGHT - detectedObjects[i].centerY;
                            break;

                        case NORMAL_LANDSCAPE:
                            break;
                    }

                    Rect rect = new Rect(detectedObjects[i].centerX - detectedObjects[i].width/2,
                                         detectedObjects[i].centerY - detectedObjects[i].height/2,
                                         detectedObjects[i].width, detectedObjects[i].height);
                    objectList.add(rect);

                    if (debugEnabled)
                    {
                        robot.tracer.traceInfo(moduleName, "[%d] %s", i, detectedObjects[i].toString());
                    }
                }
            }

            double expectedWidth = PIXY_DISTANCE_SCALE/targetDistance;
            double expectedHeight = expectedWidth*TARGET_HEIGHT_INCHES/TARGET_WIDTH_INCHES;

            if (debugEnabled)
            {
                robot.tracer.traceInfo(moduleName, "Expected Target: distance=%.1f, width=%.1f, height=%.1f",
                    targetDistance, expectedWidth, expectedHeight);
            }

            if (FILTER_ENABLED)
            {
                //
                // Filter objects out if they aren't close enough to the expected dimensions
            	// If there's more than a PERCENT_TOLERANCE error, then remove it from the arraylist
                //
                if (objectList.size() > 1)
                {
                    for (int i = 0; i < objectList.size(); i++)
                    {
                        Rect rect = objectList.get(i);
                        
                        double widthRatio = rect.width/expectedWidth;
                        double heightRatio = rect.height/expectedHeight;
                        
                        if (clamp(widthRatio, PERCENT_TOLERANCE_LOWER, PERCENT_TOLERANCE_UPPER) != widthRatio || 
                        		clamp(heightRatio, PERCENT_TOLERANCE_LOWER, PERCENT_TOLERANCE_UPPER) != heightRatio)
                        {
                        	objectList.remove(i);
                        }
                    }
                }
            }

            if (targetRect == null && objectList.size() >= 1)
            {
            	Rect maxRect = objectList.get(0);
            	for(Rect rect:objectList)
            	{
            		double size = rect.width * rect.height;
            		if(size > maxRect.width * maxRect.height)
            		{
            			maxRect = rect;
            		}
            	}

                targetRect = maxRect;

                if (debugEnabled)
                {
                    robot.tracer.traceInfo(moduleName, "===TargetRect===: x=%d, y=%d, w=%d, h=%d",
                        targetRect.x, targetRect.y, targetRect.width, targetRect.height);
                }
            }
        }

        return targetRect;
    }   //getTargetRect
    
    private double clamp(double d, double min, double max) {
    	return Math.min(Math.max(d, min), max);
    }

    public TargetInfo getTargetInfo()
    {
        TargetInfo targetInfo = null;
        Rect targetRect = getTargetRect();

        if (targetRect != null)
        {
            //CodeReview: these numbers are for last year's target, needs updating...
            //
            // Physical target width:           W = 10 inches.
            // Physical target distance 1:      D1 = 20 inches.
            // Target pixel width at 20 inches: w1 = 115
            // Physical target distance 2:      D2 = 24 inches
            // Target pixel width at 24 inches: w2 = 96
            // Camera lens focal length:        f
            //    W/D1 = w1/f and W/D2 = w2/f
            // => f = w1*D1/W and f = w2*D2/W
            // => w1*D1/W = w2*D2/W
            // => w1*D1 = w2*D2 = PIXY_DISTANCE_SCALE = 2300
            //
            // Screen center X:                 Xs = 320/2 = 160
            // Target center X:                 Xt
            // Heading error:                   e = Xt - Xs
            // Turn angle:                      a
            //    tan(a) = e/f
            // => a = atan(e/f) and f = w1*D1/W
            // => a = atan((e*W)/(w1*D1))
            //
            double targetCenterX = targetRect.x + targetRect.width/2.0;
            double targetXDistance = (targetCenterX - RobotInfo.PIXYCAM_WIDTH/2.0)*TARGET_WIDTH_INCHES/targetRect.width;
            double targetYDistance = PIXY_DISTANCE_SCALE/targetRect.width;
            double targetAngle = Math.toDegrees(Math.atan(targetXDistance/targetYDistance));
            targetInfo = new TargetInfo(targetRect, targetXDistance, targetYDistance, targetAngle);

            if (debugEnabled)
            {
                robot.tracer.traceInfo(
                    moduleName, "###TargetInfo###: xDist=%.1f, yDist=%.1f, angle=%.1f",
                    targetXDistance, targetYDistance, targetAngle);
            }
        }

//        if (targetFoundLED != null)
//        {
//            targetFoundLED.setState(targetInfo != null);
//        }
//
//        if (targetAlignedLED != null)
//        {
//            targetAlignedLED.setState(targetInfo != null && Math.abs(targetInfo.angle) <= 2.0);
//        }

        return targetInfo;
    }   //getTargetInfo

}   // class PixyVision
