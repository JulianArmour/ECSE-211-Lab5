package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.UltrasonicMedianFilter;

public class wallFollower {

	private static int BAND_CENTER = 5; //(in cm)
	private static int BAND_WIDTH = 1; //(in cm)
	private static int MAX_DELTA = 50; 
	private static double  pFactor;
	private MovementController movementControler;
	private Odometer odo;
	private double[] odoData;
	private double error;
	private static int MOTOR_SPEED = 100; 
	private UltrasonicMedianFilter USdata;
	private double distance;
	
	//constructor for wallFollower class
	public wallFollower(MovementController movementCtr, Odometer odometer,UltrasonicMedianFilter USfilter) {
		
		this.movementControler = movementCtr;
		this.odo = odometer;
		this.USdata = USfilter;
	}
	
	public void wallFollow(){
		
	odoData = odo.getXYT();	 //gets the initial XYT when wallfollower is started
	
	while(odoData[2]-5 <= odo.getXYT()[2] && odo.getXYT()[2]< odoData[2]) { 
		
		distance = USdata.getMedian();

		error = BAND_CENTER - distance;
		
		// out of bounds
		if (Math.abs(error) > BAND_WIDTH / 2) {
		//	int scaledDelta = Math.min(maxDelta, Math.abs( error * pFactor)));
			
//			int scaledDelta = Math.abs((int) (error * pFactor));
			
			int scaledDelta = (int) (Math.abs(error*1.5));
			
			if (scaledDelta > MAX_DELTA) {
				scaledDelta = MAX_DELTA;
			}
			
		//	this.scaledSpeed = scaledDelta;  why is dis here?
			
			// too close to the wall
			if (error >= 0) {
				
				movementControler.turnRight(MOTOR_SPEED,scaledDelta + 10 );
			}
			// too far away from the wall
			else {
				movementControler.turnLeft(MOTOR_SPEED,scaledDelta);
			}
		}
		else {
			
			movementControler.driveForward();
		}
		
		try {
			Thread.sleep(30);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		
		}
		
		
	}
	
	}
	
	
	
	
}
