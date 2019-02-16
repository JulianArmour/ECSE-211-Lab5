package ca.mcgill.ecse211.lab5.localization;

import ca.mcgill.ecse211.lab5.odometer.Odometer;
import lejos.hardware.Button;

/**
 * Provides methods for:
 * <ol>
 *  <li>correcting the odometer's position</li>
 *  <li>correcting the odometer's angle</li>
 *  <li>moving the robot to the (0,0)-origin oriented at 0-degrees</li>
 * </ol>
 * 
 * @author Julian Armour, Alice Kazarine
 * @since 11-02-2018
 */
public class IntersectionLocalizer {
    // a difference past this threshold consitutes a black line
    private static final int DIFFERENTIAL_THRESHOLD = 6;
    private DifferentialLightSensor diffLightSensor;
    private MovementController movCon;
    private Odometer odo;
    // the distance from the light sensor to the robot's track
    private double LTSENSOR_TO_WHEELBASE = 15; // (in cm)
    // the period between light sensor polls
    private int TIME_OUT = 20;
    // array contains the value of theta at {Y+,X+,Y-,X-}    
    private double[] intersections = { 0, 0, 0, 0 };

    public IntersectionLocalizer(DifferentialLightSensor differentialLightSensor, MovementController movementController,
            Odometer odometer) {
        this.diffLightSensor = differentialLightSensor;
        this.movCon = movementController;
        this.odo = odometer;
    }

	/**
	 * Determines the position of the robot in the tile and uses it to update the
	 * odometer with the correct position.
	 * 
	 * @see Odometer
	 */
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
		
		travelCloseToOrigin();
		movCon.turnTo(180);
		//approximately at (-5,-5) and at a 180 degree angle at this point
	}

	/**
     * Provided the robot is at the corner of a tile, and the robot is roughly
     * facing North (0-degrees). This method finds the angles at which the light
     * sensor intersects with the tile borders when the robot rotates 360-degrees.
     */
	public void getIntersections() {
	
		boolean lineDetected=false;

	//	movCon.rotateAngle(20, false,false); //to make sure the first axis it crosses is the y+
		// rotate the robot 360 degrees and record the at what angles the lines were detected
		movCon.rotateAngle(360, true,true);
		for (int i=0; i<=3; i++) {
		    // override the previous difference to prevent a false positive
			diffLightSensor.getDeltaL();
			
			while(!lineDetected) {
				int deltaL = diffLightSensor.getDeltaL();
//				System.out.println(deltaL);
				
				if (Math.abs(deltaL) >= DIFFERENTIAL_THRESHOLD) {
					lineDetected = true;
					break;
					//System.out.println("axis detected");
				}
				// delay between each sensor poll
				try {
					Thread.sleep(TIME_OUT);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			
			intersections[i] = odo.getXYT()[2];
			lineDetected=false;
			
			// wait a bit after a line has been detected to prevent it from being detected again
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
		}
		movCon.turnTo(0);
		movCon.stopMotors();
	}
	
	/**
	 * Using the 4 intersections found from {@link #getIntersections()}, this method corrects
	 *  the odometer's angle.
	 */
	public void correctAngle() {
		
		double Yangle = intersections[2]- intersections[0];   
		double headingCorrection = (Yangle/2)-intersections[2]-90; 
		
		odo.setTheta(odo.getXYT()[2] + headingCorrection); //corrects odometer's angle
		double[] currentPos = odo.getXYT();
		System.out.println("CURRENT POS: X: "+currentPos[0]+", Y: "+currentPos[1]);
	}
		
	/**
	 * Using the 4 intersections found from {@link #getIntersections()}, this method corrects
	 *  the odometer's values for X and Y position even more using the arc method
	 */
	public void correctPosition() {
		
		//correct x
		//need to calculate yAngle
		double Yangle = intersections[2]- intersections [0];
		double x = -LTSENSOR_TO_WHEELBASE*Math.cos(Math.toRadians(Yangle/2));
		odo.setX(x); //corrects odometers X position
		
		
		//correct y
		double Xangle = intersections[3]- intersections [1];
		double y = -LTSENSOR_TO_WHEELBASE*Math.cos(Math.toRadians(Xangle/2));
		odo.setY(y); //corrects odometers Y position
		
//		System.out.println(odo.getXYT()[0] + "&" + odo.getXYT()[1]);
//		Button.waitForAnyPress();
    }

	

    /**
     *  Causes the robot to move to the origin and face North (0-degrees)
     */
    public void travelToOrigin() {

        //double[] odoData = odo.getXYT();
        double angleToTurn = movCon.calculateAngle(odo.getXYT()[0], odo.getXYT()[1], 0.0, 0.0);
        System.out.println("ANGLE TO TURN: "+angleToTurn);
        movCon.turnTo(angleToTurn);

        // give the robot some time to stop
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        movCon.driveDistance(movCon.calculateDistance(odo.getXYT()[0], odo.getXYT()[1], 0.0, 0.0));
       
        //odoData[i] is changed to odo.getXYT()[i] in TravelToOrigin method
    }
     
    /**
     * Causes the robot to move to approximately (-5,-5)
     */
    public void travelCloseToOrigin() {

        //double[] odoData = odo.getXYT();
        double angleToTurn = movCon.calculateAngle(odo.getXYT()[0], odo.getXYT()[1], -5.0, -5.0);
        System.out.println("ANGLE TO TURN: "+angleToTurn);
        movCon.turnTo(angleToTurn);

        // give the robot some time to stop
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        movCon.driveDistance(movCon.calculateDistance(odo.getXYT()[0], odo.getXYT()[1], -5.0, -5.0));
       
        //odoData[i] is changed to odo.getXYT()[i] in TravelToOrigin method
    }

}
