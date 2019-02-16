package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.odometer.Odometer;

public class wallFollower {

	private static int BAND_CENTER = 5; //(in cm)
	private static int BAND_WIDTH = 1; //(in cm)
	private static int MAX_DELTA = 50; 
	private static double  pFactor;
	private MovementController movementControler;
	private Odometer odo;
	private double[] odoData;
	private double error;
	
	//constructor for wallFollower class
	public wallFollower(MovementController movementCtr, Odometer odometer) {
		
		this.movementControler = movementCtr;
		this.odo = odometer;
	}
	
	public void wallFollow(){
		
	odoData = odo.getXYT();	 //gets the initial XYT when wallfollower is started
	
	while(odoData[2]-5 <= odo.getXYT()[2] && odo.getXYT()[2]< odoData[2]) { //main class!?!?!
		

		error = BAND_CENTER - distance;
		
		// out of bounds
		if (Math.abs(error) > BAND_WIDTH / 2) {
		//	int scaledDelta = Math.min(maxDelta, Math.abs( error * pFactor)));
			
//			int scaledDelta = Math.abs((int) (error * pFactor));
			
			int scaledDelta = (int) (Math.abs(error*1.5));
			
			if (scaledDelta > MAX_DELTA) {
				scaledDelta = MAX_DELTA;
			}
			
			this.scaledSpeed = scaledDelta;
			
			// too close to the wall
			if (error >= 0) {
				MovementController.turnRight(scaledDelta + 10);
			}
			// too far away from the wall
			else {
				MovementController.turnLeft(scaledDelta);
			}
		}
		else {
			
			motorController.goStraight();
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
