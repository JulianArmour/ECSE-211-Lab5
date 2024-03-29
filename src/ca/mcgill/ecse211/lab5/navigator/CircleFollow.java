package ca.mcgill.ecse211.lab5.navigator;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import ca.mcgill.ecse211.lab5.localization.AngleCorrection;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.detectors.ColourDetector;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.ColourLightSensor;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;
import ca.mcgill.ecse211.lab5.tests.colordetection.ColorDetector;
import lejos.hardware.Sound;
import lejos.robotics.chassis.Wheel;

/**
 * Provides the methodology for causing the robot to move in a circular path around a can and collect colour data.
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
    private AngleCorrection angleCorrector;
	
	
	public CircleFollow(MovementController movementCtr, Odometer odometer, MedianDistanceSensor USfilter,
    		ColourLightSensor colorsensor, int TARGET_COLOR, URnavigator uRnavigator, AngleCorrection angleCorrection){
		this.movementController = movementCtr;
		this.odometer = odometer;
		this.medianDistanceSensor = USfilter;
		this.colourLightSensor = colorsensor;
		this.TARGET_COLOR = TARGET_COLOR;
		this.angleCorrector = angleCorrection;
		this.LTdata = new LinkedList<float[]>();
		this.urNavigator = uRnavigator;
	}
	
	/**
	 * The main method for moving the robot in a circular path and collecting colour samples
	 */
	public void followCircularPath() {
		odoBeforeWallFollow = odometer.getXYT();
		
		 double breakOutAngle = (odoBeforeWallFollow[2] + 20.0) % 360;
		 
		 
		 distance = medianDistanceSensor.getFilteredDistance();
		 movementController.driveDistance(-3, false);
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
//	     	movementController.stopMotor(true, true);
//	     	movementController.stopMotor(false, false);
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
	            angleCorrector.quickThetaCorrection();
				displayColor(canColor);
	            urNavigator.navigateToUr();
	        }
	        else {
	            //beep twice if it is not the colour we're looking for
	            Sound.twoBeeps();
	            displayColor(canColor);
	        }
	        LTdata.clear();
	}
	
	/**
     * The main method for moving the robot in a circular path and collecting colour samples
     */
    public void followCircularPathDemo() {
        odoBeforeWallFollow = odometer.getXYT();
        
         double breakOutAngle = (odoBeforeWallFollow[2] + 20.0) % 360;
         
         
         distance = medianDistanceSensor.getFilteredDistance();
         movementController.driveDistance(-3, false);
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
             // polls the ColorSensor and puts it in a list
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
            movementController.travelTo(odoBeforeWallFollow[0], odoBeforeWallFollow[1], false);
            movementController.turnTo(odoBeforeWallFollow[2]);
//          /**colourData = new float[LTdata.size()];
            colourData = new float[LTdata.size()][3];
            int i = 0;
            Iterator<float[]> iterator = LTdata.iterator();
            while (iterator.hasNext()) {
                colourData[i] = (float[]) iterator.next();
                i++;
            }
            
            int canColor = ColourDetector.verifyCan(colourData);
            displayColor(canColor);
            printColourVals(colourData);// TODO REMOVE THIS BEFORE DEMO!!!!!!!!
            LTdata.clear();
    }
	
	private void printColourVals(float[][] colourData) {
	    for (int i = 0; i < colourData.length; i++) {
            System.out.println(colourData[i][0]+"\t"+colourData[i][1]+"\t"+colourData[i][2]);
        }
    }

    /**
	 * displays on the screen which can colour has been detected
	 * @param canColor the colour to be displayed
	 */
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
        	break;
        default:
        System.out.println("Can color: N/A");
		}
	}

}
