package ca.mcgill.ecse211.lab5.localization;

import ca.mcgill.ecse211.lab5.navigator.MovementController;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.DifferentialLightSensor;
import lejos.hardware.Button;

/**
 * Provides methods for:
 * <ol>
 *  <li>correcting the odometer's position</li>
 *  <li>correcting the odometer's angle</li>
 *  <li>moving the robot to the (0,0)-origin oriented at 0-degrees</li>
 * </ol>
 * 
 * @author Alice Kazarine
 * @since Feb 23, 2019
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
     * Provided the robot is at the corner of a tile, and the robot is roughly
     * facing North (0-degrees). This method finds the angles at which the light
     * sensor intersects with the tile borders when the robot rotates 360-degrees.
     */
	public void getIntersections() {
	    
	    movCon.rotateAngle(60, false);
	
		boolean lineDetected=false;

	//	movCon.rotateAngle(20, false,false); //to make sure the first axis it crosses is the y+
		// rotate the robot 360 degrees and record the at what angles the lines were detected
		movCon.rotateAngle(360, true,true);
		for (int i=0; i<=3; i++) {
		    // override the previous difference to prevent a false positive
			diffLightSensor.getDeltaL();
			
			while(!lineDetected) {
				int deltaL = (int) diffLightSensor.getDeltaL();
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
		double headingCorrection = (Yangle/2)-intersections[2]-120;
		
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
    }
}
