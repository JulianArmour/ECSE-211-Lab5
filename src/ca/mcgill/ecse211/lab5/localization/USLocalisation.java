package ca.mcgill.ecse211.lab5.localization;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class USLocalisation implements Runnable {

	/** 
	 * Initializing variables
	 * FILTER_OUT implements a filter, functions similarly to Lab 1
	 * WALL_DISTANCE, RisingEdge_WALLDISTANCE, FallingEdge_WALLDISTANCE and XY_OFFSET are used as constants for risingEdge and fallingEdge methods
	 * MOTOR_ROTATE is set at 100 rad/s as the speed of the motors when rotating (on turns)
	 * TRACK value is obtained from testing. WHEEL_RAD is obtained from measurement (radius of the wheel) and testing
	 */
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;

	private int distance = 0;
	private int filterControl = 0;

	public static final int FILTER_OUT = 30;
	public static final int WALL_DISTANCE = 40;
	public static final int RisingEdge_WALLDISTANCE = 30;
	public static final int FallingEdge_WALLDISTANCE = 100; 
	private static final double FallingEdgeOFFSET = 46; 
	private static final double RisingEdgeOFFSET = 49;
	public static final int MOTOR_ROTATE = 100;
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 11.0;

	/**
	 * USLocalisation constructor
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK
	 * @param WHEEL_RAD
	 * @throws OdometerExceptions
	 */
	public USLocalisation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD) throws OdometerExceptions {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
		this.usSensor = ultrasonicSensor.getMode("Distance");
		this.usData = new float[usSensor.sampleSize()];
	}

	/** 
	 * Calls a run method with if statements depending on the boolean "wall" to determine 
	 * whether fallingEdge or risingEdge will be used. 
	 * If if facing a wall (wall =true), run the fallingEdge method, otherwise run the risingEdge method. 
	 */
	public void run() {
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		}

		if (Lab5.wall) { 
			fallingEdge();
		} else if (!Lab5.wall) { 
			risingEdge();
		}

	}

	/** Defining fallingEdge method, obtained from instructions off tutorial "Localization"
	 * 
	 */
	public void fallingEdge() {
		double theta;
		sensorValue();

		if (this.distance < FallingEdge_WALLDISTANCE) { //Rotates counterclockwise until the distance from the wall is greater than 100cm. 
			while (this.distance < FallingEdge_WALLDISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			//Stops motors when distance > 100, and rotates counterclockwise until distance from the wall is less than 40cm. 
			leftMotor.stop();
			rightMotor.stop();

			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			//Stop motors when distance is less than 40, and rotate clockwise until the distance from the wall is greater than 100cm. 
			leftMotor.stop();
			rightMotor.stop();
			odometer.setTheta(0); //Reset value of angle to 0 deg. 


			while (this.distance < FallingEdge_WALLDISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			//Stops the motors, and rotates clockwise until the distance from the wall is less than 40cm. 
			leftMotor.stop();
			rightMotor.stop();

			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			theta = odometer.getXYT()[2]; //Here, we record the value of the angle obtained. 
		}
		/** If the distance from the wall is greater than or equal to 100cm, 
		 * rotate counterclockwise until the distance is less than 40cm, then stop the motors, 
		 * reset the value of Theta, then rotate clockwise until the distance is greater than 100cm, then
		 * repeat the loop. 
		 * 
		 */
		else { 

			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			odometer.setTheta(0); 

			while (this.distance < FallingEdge_WALLDISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			theta = odometer.getXYT()[2]; //Here, we record the value of the angle obtained. 
		}

		// TODO change algo to work with distance sensor on the side.
		
		/** 
		 * Final part of the fallingEdge method: Adjusts the angle of the robot to 0 degrees (facing NORTH)
		 * by applying the fallingEdge method (theta1 + theta2)/2. 
		 */
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, FallingEdgeOFFSET + theta / 2.0), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, FallingEdgeOFFSET + theta / 2.0), false);
		odometer.setTheta(0);
		leftMotor.stop();
		rightMotor.stop();
	}

	/** 
	 * Defining risingEdge method, also obtained from the "Localization" tutorial. 
	 * It consists of rotating counterclockwise until the distance is less than RisingEdge_WALLDISTANCE 
	 * than until the distance is greater than WALL_DISTANCE
	 * in a loop while recording the angle, and then rotating clockwise with the same conditions and recording that second angle. 
	 * Performing a computation of (theta1+theta2)/2 to turn the robot to 0 degrees (facing NORTH)
	 */
	public void risingEdge() {
		double theta;
		sensorValue();
		if (this.distance > RisingEdge_WALLDISTANCE) {

			while (this.distance > RisingEdge_WALLDISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			while (this.distance < WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			odometer.setTheta(0); 

			while (this.distance > RisingEdge_WALLDISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			while (this.distance < WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			theta = odometer.getXYT()[2]; 

		} else {
			while (this.distance < WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			odometer.setTheta(0); 

			while (this.distance > RisingEdge_WALLDISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			while (this.distance < WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			theta = odometer.getXYT()[2]; 
		}

		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta / 2.0 - RisingEdgeOFFSET), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta / 2.0 - RisingEdgeOFFSET), false);
		odometer.setTheta(0);
		leftMotor.stop();
		rightMotor.stop();
	}


	private void sensorValue() {
		usSensor.fetchSample(usData, 0);
		int dist = (int) Math.abs(usData[0] * 100.0);
		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (dist >= 255 && filterControl < FILTER_OUT) {
			filterControl++;
		} else if (dist >= 255) {
			this.distance = dist;
		} else {
			filterControl = 0;
			this.distance = dist;
		}
	}

	/**
	 * convertDistance() and convertAngle() methods convert the distance and angle needed
	 * as rotations of the wheel. 
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
