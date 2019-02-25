package ca.mcgill.ecse211.lab5.navigator;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.detectors.ColourDetector;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.ColourLightSensor;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;
import lejos.hardware.Sound;
import lejos.robotics.chassis.Wheel;

/**
 * Provides the methodology for causing the robot to move in a circular path around a can.
 * @author Alice Kazarine
 * @since Feb 24, 2019
 */
public class CircleFollow {
		
	private static MovementController movementController;
	private static Odometer odometer;
	private static MedianDistanceSensor medianDistanceSensor;
	private static ColourLightSensor colourLightSensor;
	
	private static double WHEEL_BASE = 11.3;
	private static double Wheel_RADIUS = 2.1;
	private static int TARGET_COLOR;
    private double distance;
    private float[][] colourData;
	
	 private double[] odoBeforeWallFollow;
	 private LinkedList<float[]> LTdata;
    private URnavigator urNavigator;
	
	
	public CircleFollow(MovementController movementCtr, Odometer odometer, MedianDistanceSensor USfilter,
    		ColourLightSensor colorsensor, int TARGET_COLOR, URnavigator uRnavigator){
		this.movementController = movementCtr;
		this.odometer = odometer;
		this.medianDistanceSensor = USfilter;
		this.colourLightSensor = colorsensor;
		this.TARGET_COLOR = TARGET_COLOR;
		this.LTdata = new LinkedList<float[]>();
		this.urNavigator = uRnavigator;
	}
	
	public void followCircularPath() {
		odoBeforeWallFollow = odometer.getXYT();
		
		 double breakOutAngle = odoBeforeWallFollow[2] + 20.0;
		 
		 
		 distance = medianDistanceSensor.getFilteredDistance();
		 movementController.driveDistance(-4, false);
         if (distance > 4) {
         	
         	movementController.rotateAngle(90, false);
         	movementController.driveDistance((distance-4), false);
         	movementController.rotateAngle(90, true);
         } else {
             movementController.rotateAngle(90, false);
             movementController.driveDistance(-(4-distance), false);//move backwards
             movementController.rotateAngle(90, true);
         }
         
         movementController.goInCircularPath();
		 
         //((odometer.getXYT()[2] - breakOutAngle + 360) % 360);
	     while (((odometer.getXYT()[2] - breakOutAngle + 360) % 360) > 20) {
	         // polls the ColorSensor and puts it in an array
             float[] colorData = colourLightSensor.fetchColorSamples();
             
             if (colorData[0] > 0.001 && colorData[1] > 0.001 && colorData[2] > 0.001) {
                 LTdata.add(colorData);
             }
             
             
             try {
                 Thread.sleep(100);
             } catch (InterruptedException e) {
                 e.printStackTrace();

             }
	        	
	        }
	     	movementController.stopMotor(true, true);
	     	movementController.stopMotor(false, false);
	     	movementController.travelTo(odoBeforeWallFollow[0], odoBeforeWallFollow[1], false);
	        movementController.turnTo(odoBeforeWallFollow[2]);
//	        /**colourData = new float[LTdata.size()];
	        colourData = new float[LTdata.size()][3];
            int i = 0;
            for (Iterator<float[]> iterator = LTdata.iterator(); iterator.hasNext();) {
                colourData[i] = (float[]) iterator.next();
                i++;
            }
            int canColor = ColourDetector.verifyCan(colourData);
	        if(TARGET_COLOR == canColor) {
	            //beep once if it is the colour we're looking for
	            Sound.beep();
	            displayColor(canColor);

	            urNavigator.navigateToUr();
	        }
	        else {
	            //beep twice if it is not the colour we're looking for
	            Sound.twoBeeps();
	            displayColor(canColor);
	        }
		
	}
	private void displayColor(int canColor) {
		switch (canColor) {
        case 4:
        	System.out.println("Can color: Red");
        	break;
        case 3:
        	System.out.println("Can color: Yellow");
        	break;
        case 2:
        	System.out.println("Can color: Green");
        	break;
        case 1: 
        	System.out.println("Can color: Blue");
        	default:
        	System.out.println("Can color: N/A");
		}
	}

}
