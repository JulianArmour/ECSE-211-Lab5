package ca.mcgill.ecse211.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

	private OdometerData odoData;
	private static Odometer odo = null; // Returned as singleton

	/** Variable declaration
	 * left and rightMotorTachoCount is a counter for the number of rotations performed 
	 * distL and distR are used to compute the robot's new position
	 * deltaD and deltaT are used to compute the overall displacement and change in angle
	 * x, y, theta are set to 0 as the initial position of the robot
	 * dx and dy are used to update robot's position
	 * TRACK and WHEEL_RAD, like Lab 2, are parameters set to turn and move the robot forward
	 * The odometer updates every ODOMETER_PERIOD, in other words every 25ms
	 */
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private int lastLeftMotorTachoCount;
	private int lastRightMotorTachoCount;
	private double x = 0;
	private double	 y = 0; 
	private double theta = 0;
	private double dx, dy;
	private double distL,distR, deltaD, deltaT;

	private final double TRACK;
	private final double WHEEL_RAD;

	private static final long ODOMETER_PERIOD = 25; 

	/**
	 * This is the default constructor of this class. It initiates all motors and variables once.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
		// manipulation methods
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		odoData.setXYT(0, 0, 0);

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;

		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
					throws OdometerExceptions {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be used only if an
	 * odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods provided from the
	 * OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();

			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();

			/** The following equations are written to calculate the new robot positions 
			 * based on tachometer counts. 
			 * LeftDist and RightDist equations compute the wheel displacements. 
			 * deltaD calculates the overall displacement
			 * deltaT calculates the change in angle in radians, then degrees. 
			 * dx and dy calculate the x and y components of displacement, respectively. 
			 * Finally, the x and y and theta position values are updated. 
			 */

			distL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - lastLeftMotorTachoCount)/180; 
			distR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - lastRightMotorTachoCount)/180; 
			lastLeftMotorTachoCount = leftMotorTachoCount; 
			lastRightMotorTachoCount = rightMotorTachoCount;
			deltaD = 0.5 * (distL + distR); 
			deltaT = (distL - distR) / TRACK; 
			theta += deltaT; 
			dx = deltaD * Math.sin(theta); 
			dy = deltaD * Math.cos(theta); 
			x = x + dx; 
			y = y + dy;		


			/** Here, the odometer values are updated with the calculations made above, 
			 * using dx, dy, and deltaT as the odometer values
			 */

			odo.update(dx, dy, deltaT * 180/Math.PI);


			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}
}