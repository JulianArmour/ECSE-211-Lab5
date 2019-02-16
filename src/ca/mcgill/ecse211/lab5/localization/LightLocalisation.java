package ca.mcgill.ecse211.lab5.localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/** 
 * Initializing variables and left and right Motors, 
 * backwardDistance is set to 13cm as the distance the robot backs up to align on the Y-axis
 * middleDistance is set to 8cm as the distance the robot backs up to align on the X-axis. 
 * Constants TRACK and WHEEL_RAD are imported from the Lab4 Java class. 
 *
 */
public class LightLocalisation implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private SampleProvider color;
	private float[] lsData;
	private float colourIntensity;

	private static final int backwardDistance = 13;
	private static final int middleDistance = 12;
	private static final double LightIntensityThreshold = 0.2;
	private double TRACK;
	private double WHEEL_RAD;

	/**
	 * LightLocalisation constructor
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK
	 * @param WHEEL_RAD
	 */
	public LightLocalisation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

		EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("A")); //Port assigned to 4 for the lightSensor
		color = lightSensor.getMode("Red"); //LightSensor operates on the "Red" mode that measures Red light reflection intensity
		lsData = new float[lightSensor.sampleSize()];
	}

	/** Run method calls the localization constructor
	 * 
	 */
	public void run() {
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		}
		localization();
	}

	public void localization() {
		sensorValue();
		/** 
		 * Robot advances until a line is detected, stops the motors then turns 90 degrees clockwise
		 * and advances until another line is detected. Once a line is detected, the motos are stopped and the robot
		 * moves backwards by a distance of backwardDistance, then turns counterclockwise and backs up by a distance
		 * of middleDistance to finally align on the (0,0) coordinates facing the 0 degree direction. 
		 */
		while (colourIntensity > LightIntensityThreshold) {
			leftMotor.forward();
			rightMotor.forward();
			sensorValue();
		}
		Sound.beep(); //Implemented to ensure lightSensor detects the navy/black line. 
		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
		sensorValue();

		while (colourIntensity > LightIntensityThreshold) {
			leftMotor.forward();
			rightMotor.forward();
			sensorValue();
		}
		//Here, a navy/black line has been detected
		Sound.beep();
		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.rotate(convertDistance(WHEEL_RAD, -backwardDistance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -backwardDistance), false);
		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);

		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.rotate(convertDistance(WHEEL_RAD, -middleDistance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -middleDistance), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * The sensorValue() method returns the Red light reflection intensity
	 * @return void
	 */
	public void sensorValue() {
		color.fetchSample(lsData, 0);
		this.colourIntensity = lsData[0];
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
