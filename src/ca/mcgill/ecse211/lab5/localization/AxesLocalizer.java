package ca.mcgill.ecse211.lab5.localization;

import ca.mcgill.ecse211.lab5.navigator.MovementController;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.DifferentialLightSensor;

public class AxesLocalizer {
	
	private MovementController movCon;
	private Odometer odo;
	private DifferentialLightSensor diffLightSensor;
	private static int DIFFERENTIAL_THRESHOLD = 6;
	private static double LTSENSOR_TO_WHEELBASE = 11.9;
	private static int TIME_OUT = 20;
	
	
	//constructor
	public AxesLocalizer(MovementController movementController, Odometer odometer, 
			DifferentialLightSensor diffLightSensor) {
		
		this.movCon=movementController;
		this.odo=odometer;
		this.diffLightSensor=diffLightSensor;
		
		
	}

	public void estimatePosition() {


		// go forward until light sensor detects the x-axis
		movCon.driveForward();
		boolean lineDetected = false;

		// keep checking for a black line
		while (!lineDetected) {
			int deltaL = diffLightSensor.getDeltaL();
		//	System.out.println(deltaL);
			
			try {
				Thread.sleep(TIME_OUT);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			if (Math.abs(deltaL) > DIFFERENTIAL_THRESHOLD) {
				lineDetected = true;
				
			}
		}
		
		// a line has been found, stop the motos and set the odometer's y-position
		movCon.stopMotors();
		odo.setY(LTSENSOR_TO_WHEELBASE);   //should the constant be in meters??

		movCon.driveDistance(-20); //drives backwards
		movCon.rotateAngle(90, true); //rotates parallel to x-axis
		
		
		lineDetected = false;
		movCon.driveForward();
		diffLightSensor.getDeltaL();
		
		// check for the next line
		while (!lineDetected) {
			int deltaL = diffLightSensor.getDeltaL();
//           System.out.println(deltaL);
			
			try {
				Thread.sleep(TIME_OUT);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			if (Math.abs(deltaL) > DIFFERENTIAL_THRESHOLD) {
				lineDetected = true;
				
				
			}
		}
		
		// the x-axis has been detected, set the odometer's x-position
		movCon.stopMotors();
		odo.setX(LTSENSOR_TO_WHEELBASE); //should the constant be in meters??

//		double[] currentPos = odo.getXYT();
//		System.out.println("CURRENT POS: X: "+currentPos[0]+", Y: "+currentPos[1]);
		
		movCon.travelCloseToOrigin(odo);
		movCon.turnTo(180);
		//approximately at (-5,-5) and at a 180 degree angle at this point
	}
}
