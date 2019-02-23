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
	 private List<float[]> LTdata;
	
	
	public CircleFollow(MovementController movementCtr, Odometer odometer, MedianDistanceSensor USfilter,
    		ColourLightSensor colorsensor, int TARGET_COLOR){
		this.movementController = movementCtr;
		this.odometer = odometer;
		this.medianDistanceSensor = USfilter;
		this.colourLightSensor = colorsensor;
		this.TARGET_COLOR = TARGET_COLOR;
		this.LTdata = new LinkedList<float[]>();
		
	}
	
	public void followCircularPath() {
		medianDistanceSensor.flush();
		odoBeforeWallFollow = odometer.getXYT(); 
		
		 double breakOutAngle = odoBeforeWallFollow[2] + 40.0;
		 
		 
		 distance = medianDistanceSensor.getFilteredDistance();
         
         if (distance > 5) {
         	
         	movementController.rotateAngle(90, false);
         	movementController.driveDistance((distance-5), false);
         	movementController.rotateAngle(90, true);
         } else {
             movementController.rotateAngle(90, true);
             movementController.driveDistance((distance+5), false);
             movementController.rotateAngle(90, true);
         }
         
         movementController.goInCircularPath();
		 
	     while (Math.abs(odometer.getXYT()[2] - breakOutAngle) > 20) {
	        	
	        	// polls the ColorSensor and puts it in an array
	        	float[] colorData = colourLightSensor.fetchColorSamples();
	        	
	        	if (colorData[0] > 0.001 || colorData[1] > 0.001 || colorData[2] > 0.001) {
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
	        colourData =  (float[][]) LTdata.toArray();
	        colourData = new float[LTdata.size()][3];
	        int i = 0;
	        for (Iterator<float[]> iterator = LTdata.iterator(); iterator.hasNext();) {
	            colourData[i] = (float[]) iterator.next();
	            i++;
	        }
	        
	        if(ColourDetector.verifyCan(colourData, TARGET_COLOR)) {
	            //beep once
	            Sound.beep();
	        }
	        else {
	            //beep twice
	            Sound.twoBeeps();
	        }
		
	}

}
