package ca.mcgill.ecse211.lab5.localization;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.navigator.MovementController;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.DifferentialLightSensor;

/**
 * Provides the methods for the beginning of the seconds phase of localization
 * @author Alice Kazarine
 * @since Feb 24, 2019
 */
public class AxesLocalizer {
	
	private MovementController movCon;
	private Odometer odo;
	private DifferentialLightSensor rDiffLightSensor;
    private DifferentialLightSensor lDiffLightSensor;
	private static float DIFFERENTIAL_THRESHOLD = 6.0f;
	private static double LTSENSOR_TO_WHEELBASE = 11.9;
	private static int TIME_OUT = 20;
	
	
	/**
	 * 
	 * @param movementController the {@link MovementController}
	 * @param odometer the {@link Odometer}
	 * @param lDifferentialLightSensor the left {@link DifferentialLightSensor}
	 * @param rDifferentialLightSensor the right {@link DifferentialLightSensor}
	 */
	public AxesLocalizer(MovementController movementController, Odometer odometer, 
	        DifferentialLightSensor lDifferentialLightSensor, DifferentialLightSensor rDifferentialLightSensor) {
		this.movCon=movementController;
		this.odo=odometer;
		this.lDiffLightSensor = lDifferentialLightSensor;
		this.rDiffLightSensor = rDifferentialLightSensor;
	}

	/**
	 * Moves the robot north until a black line is detected, which sets the y-position of the odometer.
	 * Then moves the robot back, turns east, and moves east until a black line is detected, which sets the
	 * x-position of the odometer.
	 */
	public void estimatePosition() {

	    lDiffLightSensor.flush();
		// go forward until light sensor detects the x-axis
	    movCon.driveDistance(Lab5.TILE_SIZE * 0.3, false);
		movCon.driveForward();

		// keep checking for a black line
		float deltaL;
		
		while ((deltaL = lDiffLightSensor.getDeltaL()) < DIFFERENTIAL_THRESHOLD) {
		    try {
                Thread.sleep(TIME_OUT);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
		}
		
		// a line has been found, stop the motos and set the odometer's y-position
		movCon.stopMotors();
		System.out.println("saw 1st line, difference of "+deltaL);
		odo.setY(LTSENSOR_TO_WHEELBASE);   //should the constant be in meters??

		movCon.driveDistance(-20); //drives backwards
		movCon.rotateAngle(90, true); //rotates parallel to x-axis
		
		rDiffLightSensor.flush();
		
		movCon.driveForward();
		
		// check for the next line
		while ((deltaL = rDiffLightSensor.getDeltaL()) < DIFFERENTIAL_THRESHOLD) {
            try {
                Thread.sleep(TIME_OUT);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
		
		// the x-axis has been detected, set the odometer's x-position
		movCon.stopMotors();
		System.out.println("saw 2nd line, difference of "+deltaL);
		odo.setX(LTSENSOR_TO_WHEELBASE);

		travelCloseToOrigin();
		movCon.turnTo(180);
		
	}
	
	/**
     * Causes the robot to move to near the origin for {@link IntersectionLocalizer#getIntersections()}
     */
    public void travelCloseToOrigin() {

        //double[] odoData = odo.getXYT();
        double angleToTurn = movCon.calculateAngle(odo.getXYT()[0], odo.getXYT()[1], -1.0, -1.0);
//        System.out.println("ANGLE TO TURN: "+angleToTurn);
        movCon.turnTo(angleToTurn);

        // give the robot some time to stop
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        movCon.driveDistance(movCon.calculateDistance(odo.getXYT()[0], odo.getXYT()[1], -1.0, -1.0));
       
        //odoData[i] is changed to odo.getXYT()[i] in TravelToOrigin method
    }
}
