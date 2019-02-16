package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.display.Display;
import ca.mcgill.ecse211.lab5.localization.LightLocalisation;
import ca.mcgill.ecse211.lab5.localization.USLocalisation;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LocalizationTest {

	/** Initialize variables for radius of the wheel and track, assign ports for left and rightMotor 
	 * Define boolean "wall" to simply lightLocalizer method of assigning fallingEdge or risingEdge constructors. 
	 */
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 11.75;
	public static boolean wall; 

	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice;

		Odometer odometer = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		USLocalisation usLocalizer = new USLocalisation(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		LightLocalisation lightLocalizer = new LightLocalisation(leftMotor, rightMotor, TRACK, WHEEL_RAD);

		Display odometryDisplay = new Display(lcd);
		do {
			/**
			 * Clears the LCD and displays the main question: Do we want Rising Edge or Falling Edge?
			 * On the left is RisingEdge method is needed, or on the right if FallingEdge method is required.
			 */
			lcd.clear();

			lcd.drawString("< Left |  Right >", 0, 0);
			lcd.drawString("       |         ", 0, 1);
			lcd.drawString("Rising |  Falling", 0, 2);
			lcd.drawString("edge   |  edge   ", 0, 3);
			lcd.drawString("       | 		 ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
			if (buttonChoice == Button.ID_ESCAPE) { //Gives the user the option to opt out of the menu before executing a function
				System.exit(0);
			}
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
		/** If left button is pressed, run ultrasonicLocalizer taking into account that we're not facing the wall,
		 * hence executing the risingEdge method. Once the risingEdge method is executed, run the LightLocalizer
		 * However, if the right button is pressed, we run the ultrasonicLocalizer while assigning to the boolean "wall" the 
		 * value "true", basically telling ultrasonicLocalizer to run the method fallingEdge. Then proceed by running LightLocalizer
		 */
		if (buttonChoice == Button.ID_LEFT) { 
			wall = false;
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			
			usLocalizer.run(); 
			buttonChoice = Button.waitForAnyPress();
			if (buttonChoice == Button.ID_ESCAPE) {
				System.exit(0);
			}
			else if (buttonChoice == Button.ID_ENTER) {
				lightLocalizer.run();
			}

		} else if (buttonChoice == Button.ID_RIGHT){
			wall = true;
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			usLocalizer.run();
			buttonChoice = Button.waitForAnyPress();
			if (buttonChoice == Button.ID_ESCAPE) {
				System.exit(0);
			}
			else if (buttonChoice == Button.ID_ENTER) {
				lightLocalizer.run();
			}

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}