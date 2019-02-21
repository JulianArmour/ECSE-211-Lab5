package ca.mcgill.ecse211.lab5.tests;

import ca.mcgill.ecse211.lab5.display.Display;

import ca.mcgill.ecse211.lab5.localization.USAngleCorrector;

import ca.mcgill.ecse211.lab5.localization.angleCorrection;
import ca.mcgill.ecse211.lab5.navigator.LLnavigator;
import ca.mcgill.ecse211.lab5.navigator.MovementController;
import ca.mcgill.ecse211.lab5.navigator.SearchNavigator;
import ca.mcgill.ecse211.lab5.navigator.wallFollower;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.odometer.OdometerExceptions;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.ColourLightSensor;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.DifferentialLightSensor;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;

public class SearchAlgoTest {
	//
    // Global Parameters
    private static final int LLx = 3;
    private static final int LLy = 3;
    private static final int URx = 7;
    private static final int URy = 7;
    private static final int SC = 0;
    private static final int TR = 0;
    
    // physical values for LLx, LLy, URx, URy
    private static double PLLx;
    private static double PLLy;
    private static double PURx;
    private static double PURy;

	/** Initialize variables for radius of the wheel and track, assign ports for left and rightMotor 
	 * Define boolean "wall" to simply lightLocalizer method of assigning fallingEdge or risingEdge constructors. 
	 */
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	/** The tile's length. */
    public static final double TILE_SIZE = 30.48;
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 11.9;
	public static boolean wall;
	
    private static MovementController movementController;
    private static LLnavigator llNavigator;
    private static Port sideUSPort;
    private static EV3UltrasonicSensor sideUltrasonicSensor;
    private static SensorMode sideDistanceProvider;
    private static float[] sideUSSample;
    private static Port backLeftLSPort;
    private static EV3ColorSensor backLeftLS;
    private static SensorMode backLeftLSProvider;
    private static float[] backLeftLSSample;
    private static Port backRightLSPort;
    private static EV3ColorSensor backRightLS;
    private static SensorMode backRightLSProvider;
    private static float[] backRightLSSample;
    private static Port sideLSPort;
    private static EV3ColorSensor sideLS;
    private static SensorMode sideLSProvider;
    private static float[] sideLSSample;
    

    private static Odometer odometer;
    private static USAngleCorrector usLocalizer;
    private static DifferentialLightSensor leftDifferentialLightSensor;
    private static DifferentialLightSensor rightDifferentialLightSensor;
    private static angleCorrection angleCorrection;
    private static SearchNavigator searchNavigator;
    private static wallFollower wallFollower;
    private static MedianDistanceSensor medianDistanceSensor;
    private static ColourLightSensor colourLightSensor;

	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice;
		
		// coordinate conversion
		PLLx = TILE_SIZE * (double) LLx;
		PLLy = TILE_SIZE * (double) LLy;
		PURx = TILE_SIZE * (double) URx;
		PURy = TILE_SIZE * (double) URy;
		
		// set up side ultrasonic sensor
        sideUSPort = LocalEV3.get().getPort("S1");
        sideUltrasonicSensor = new EV3UltrasonicSensor(sideUSPort);
        sideDistanceProvider = sideUltrasonicSensor.getMode("Distance");
        sideUSSample = new float[sideDistanceProvider.sampleSize()];
        
        // set up back-left light sensor
        backLeftLSPort = LocalEV3.get().getPort("S2");
        backLeftLS = new EV3ColorSensor(backLeftLSPort);
        backLeftLSProvider = backLeftLS.getMode("Red");
        backLeftLSSample = new float[backLeftLSProvider.sampleSize()];
        
        // set up side light sensor
        sideLSPort = LocalEV3.get().getPort("S4");
        sideLS= new EV3ColorSensor(sideLSPort);
        sideLSProvider = sideLS.getMode("RGB");
        sideLSSample = new float[sideLSProvider.sampleSize()];
        
        
        // set up back-right light sensor
        backRightLSPort = LocalEV3.get().getPort("S3");
        backRightLS = new EV3ColorSensor(backRightLSPort);
        backRightLSProvider = backRightLS.getMode("Red");
        backRightLSSample = new float[backRightLSProvider.sampleSize()];
        
      
        odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
        Thread odoThread= new Thread(odometer);
        odoThread.start();
        movementController = new MovementController(leftMotor, rightMotor, WHEEL_RAD, TRACK, odometer);
        usLocalizer = new USAngleCorrector(movementController, odometer, medianDistanceSensor);
        
        leftDifferentialLightSensor = new DifferentialLightSensor(backLeftLSProvider, backLeftLSSample);
        rightDifferentialLightSensor = new DifferentialLightSensor(backRightLSProvider, backRightLSSample);
        
      
        angleCorrection = new angleCorrection(rightDifferentialLightSensor, leftDifferentialLightSensor, movementController, odometer);
        

        colourLightSensor = new ColourLightSensor(sideLSProvider, sideLSSample);
        medianDistanceSensor = new MedianDistanceSensor(sideDistanceProvider, sideUSSample, odometer);
        wallFollower = new wallFollower(movementController, odometer, medianDistanceSensor, colourLightSensor, TR);
     
        searchNavigator = new SearchNavigator(odometer, movementController, LLx, LLy, URx, URy, medianDistanceSensor, wallFollower,  angleCorrection);
       
        
        
        
		do {
			/**
			 * Clears the LCD and displays the main question: Do we want Rising Edge or Falling Edge?
			 * On the left is RisingEdge method is needed, or on the right if FallingEdge method is required.
			 */
			lcd.clear();

			lcd.drawString("Right > Run test", 0, 0);
			
			buttonChoice = Button.waitForAnyPress();
		}
		while (buttonChoice != Button.ID_RIGHT);
		
		/** If left button is pressed, run ultrasonicLocalizer taking into account that we're not facing the wall,
		 * hence executing the risingEdge method. Once the risingEdge method is executed, run the LightLocalizer
		 * However, if the right button is pressed, we run the ultrasonicLocalizer while assigning to the boolean "wall" the 
		 * value "true", basically telling ultrasonicLocalizer to run the method fallingEdge. Then proceed by running LightLocalizer
		 */
		if (buttonChoice == Button.ID_RIGHT) {
			odometer.setXYT(PLLx, PLLy, 0);
			searchNavigator.searchPath();
			
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
		
	}


}
