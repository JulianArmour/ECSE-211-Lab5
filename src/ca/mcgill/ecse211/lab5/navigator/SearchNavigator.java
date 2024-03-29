package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.localization.AngleCorrection;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

//takes in integer, not physical measures
public class SearchNavigator implements TimerListener {


	private double TILE_LENGTH = Lab5.TILE_SIZE;
	private static final double DESTINATION_THRESHOLD = 4.0;
	private static final int CAN_SCAN_PERIOD = 50;
	private Odometer odometer;
	private MovementController movementController;
	private MedianDistanceSensor USdata;
	private CircleFollow circleFollower;
	private AngleCorrection angleCorrector;
	private int llX;
	private int llY;
	private int urX;
	private int urY;
	
	private int deltaY;
	private int deltaX;
	private double Xdistance;
	private double Ydistance;
	boolean canDetected = false;
	private double[] destination;
	double[] currentPos;
	Timer canTimer;
    private boolean canFollowing;

	public SearchNavigator(Odometer odometer, MovementController movementController, int llX, int llY, int urX, int urY,
			MedianDistanceSensor USdata, CircleFollow circleFollow, AngleCorrection angleCorrector) {

		this.odometer = odometer;
		this.movementController = movementController;
		this.USdata = USdata;
		this.angleCorrector = angleCorrector;
		this.llX = llX;
		this.llY = llY;
		this.urX = urX;
		this.urY = urY;
		this.circleFollower = circleFollow;
		this.currentPos = new double[3];
		this.canFollowing = false;
	}

	/**
	 * Causes the robot to travel in a spiral shaped path. During this it checks for cans.
	 * @see SearchNavigator#timedOut();
	 */
	public void searchPath() {
		
		// start an angle correction thread
		System.out.println("Beggining -> Xodo: " +odometer.getXYT()[0] + "Yodo: " + odometer.getXYT()[1]);
		deltaY =  (urY - llY);
		deltaX =  (urX - llX);

		movementController.driveDistance(-TILE_LENGTH * 0.8);
		movementController.turnTo(90);
		
		
		
		// hardcoded part on x axis

		// create the timer
		canTimer = new Timer(CAN_SCAN_PERIOD, this);
		


		Xdistance = (deltaX + 1.1)*TILE_LENGTH;

		currentPos = odometer.getXYT();
		destination = new double[] {currentPos[0] + Xdistance, currentPos[1], currentPos[2] };
		System.out.println("Xodo: " +odometer.getXYT()[0] + "Yodo: " + odometer.getXYT()[1]);
		System.out.println("destX: " +destination[0] + "destY: " + destination[1]);
		
		angleCorrector.quickThetaCorrection();
		movementController.travelTo(destination[0], destination[1],true);

		// check for cans while driving
		canTimer.start();
		
		//pause until robot reaches destination
		while (distanceToDestination() > DESTINATION_THRESHOLD || canFollowing) {
			try {
				Thread.sleep(200);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		// stop looking for cans
		canTimer.stop();
		// robot is now within DESTINATION_THRESHOLD, move remaining distance
		System.out.println("arriving in 1 sec");
		System.out.println("Xodo: " +odometer.getXYT()[0] + "Yodo: " + odometer.getXYT()[1]);
		movementController.travelTo(destination[0], destination[1], false);
		System.out.println("arrived at destination");
		System.out.println("Xodo: " +odometer.getXYT()[0] + "Yodo: " + odometer.getXYT()[1]);
		movementController.turnTo(90);
		


		// for loop of remaining path
		for (int n = deltaX, m = deltaY, i = 0; n > 0 & m > 0 & i < 10; n--, m--, i++) {
		    int adjustDist = 5;
			movementController.rotateAngle(90, false);
			movementController.driveDistance(adjustDist, false);
			
			Ydistance = (((double)n + 1.0) * TILE_LENGTH) - adjustDist; //TODO: tweak


			if(movementController.roundAngle()==0) {
				System.out.println("Angle 0 detected, going up");
				double[] currentPos = odometer.getXYT();
				destination[0] = currentPos[0];
				destination[1] = currentPos[1]+Ydistance;
				destination[2] = currentPos[2];
				System.out.println("next Y dest is X: " +destination[0] + "Y: " + destination[1]);
			}
			else if(movementController.roundAngle() == 180) {
				System.out.println("Angle 180 detected, going down");
				double[] currentPos = odometer.getXYT();
				destination[0] = currentPos[0];
				destination[1] = currentPos[1]-Ydistance;
				destination[2] = currentPos[2];
				System.out.println("next Y dest is X: " + destination[0] + "Y: " + destination[1]);
			}
			
			// perform a quick angle correction
            
			angleCorrector.quickThetaCorrection();
			movementController.driveDistance(adjustDist); // so we don't detect the can again
			movementController.travelTo(destination[0], destination[1],true);
			
			// check for cans
			canTimer.start();

			// pause until destination is reached
			while (distanceToDestination() > DESTINATION_THRESHOLD || canFollowing ) {
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			// stop looking for cans
			canTimer.stop();
			// robot is now within DESTINATION_THRESHOLD, move remaining distance
			double currentAngle = movementController.roundAngle();
			movementController.travelTo(destination[0], destination[1], false);
			System.out.println("Xodo: " +odometer.getXYT()[0] + "Yodo: " + odometer.getXYT()[1]);
			System.out.println("arrived at next Ydestination");
			movementController.turnTo(currentAngle);
			System.out.println("will now travel at theta:" + currentAngle);
			// move in y direction after reaching destination

			movementController.rotateAngle(90, false);
			movementController.driveDistance(adjustDist, false);
		

			
			
			//starts moving parallel to the x-axis
			Xdistance = (((double)m + 1) * TILE_LENGTH) - adjustDist;//TODO: tweak


			if(movementController.roundAngle() == 90) {
				double [] currentPos = odometer.getXYT();
				destination[0]=currentPos[0] + Xdistance;
				destination[1]=currentPos[1];
				destination[2]= currentPos[2];
				System.out.println("next X dest is X: " +destination[0] + "Y: " + destination[1]);
			}
			else if (movementController.roundAngle() == 270) {
				double [] currentPos = odometer.getXYT();
				destination[0]=currentPos[0] - Xdistance;
				destination[1]=currentPos[1];
				destination[2]= currentPos[2];
				System.out.println("next X dest is X: " +destination[0] + "Y: " + destination[1]);
			}
			
			angleCorrector.quickThetaCorrection();
			movementController.driveDistance(adjustDist); // so we don't detect the can again
			movementController.travelTo(destination[0], destination[1],true);
			canTimer.start();

			// pause until destination is reached
			while (distanceToDestination() > DESTINATION_THRESHOLD || canFollowing) {
				try {
					Thread.sleep(300);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			// stop looking for cans
			canTimer.stop();
			// robot is now within DESTINATION_THRESHOLD, move remaining distance
			currentAngle = odometer.getXYT()[2];
			movementController.travelTo(destination[0], destination[1], false);
			System.out.println("arrived at next Xdestination");
			movementController.turnTo(currentAngle);
		}
		//
	}

	/**
	 * @return the distance between the robot and the current destination
	 */
	private double distanceToDestination() {
		double[] curPos = odometer.getXYT();
		double dX = destination[0] - curPos[0];
		double dY = destination[1] - curPos[1];
		return Math.sqrt(dX * dX + dY * dY);
	}

	/**
	 * {@link Timer} method for checking if a can is directly beside the robot. If there is a can
	 * it causes the robot to rotate around it and collect colour samples.
	 * @see CircleFollow#followCircularPath()
	 */
	@Override
	public void timedOut() {

		// if US sensor detects a can
		if (USdata.getFilteredDistance() < 20) {
			//System.out.println(canDist);
			movementController.stopMotors();
			canFollowing = true;
//			currentPos = odometer.getXYT();

//			canDetected = true; // maybe use this to influence the for loop to interrupt


			// goes into wallfollowing mode and collects colour data
			circleFollower.followCircularPath();
			
//			movementController.travelTo(currentPos[0], currentPos[1], false);
//			movementController.turnTo(currentPos[2]);
			// Note: at this point the robot is back to where it was before wall-following

			// angle correction
			angleCorrector.quickThetaCorrection();
			
			// move away from the can so it's not detected again
			movementController.driveDistance(TILE_LENGTH*0.8, false);
			USdata.flush();
			
			// keep moving remaining distance/is this the right destination?
			movementController.travelTo(destination[0], destination[1],true);
			canFollowing = false;
		}
	

	}
}
