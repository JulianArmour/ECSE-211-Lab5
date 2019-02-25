package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.display.Display;
import ca.mcgill.ecse211.lab5.localization.AxesLocalizer;
import ca.mcgill.ecse211.lab5.localization.IntersectionLocalizer;
import ca.mcgill.ecse211.lab5.localization.USAngleCorrector;
import ca.mcgill.ecse211.lab5.localization.angleCorrection;
import ca.mcgill.ecse211.lab5.navigator.CircleFollow;
import ca.mcgill.ecse211.lab5.navigator.LLnavigator;
import ca.mcgill.ecse211.lab5.navigator.MovementController;
import ca.mcgill.ecse211.lab5.navigator.SearchNavigator;
import ca.mcgill.ecse211.lab5.navigator.URnavigator;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.odometer.OdometerExceptions;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.ColourLightSensor;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.DifferentialLightSensor;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;
import ca.mcgill.ecse211.lab5.tests.colordetection.ColorDetector;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;

public class Lab5 {
    // Global Parameters
    private static final int LLx = 1;
    private static final int LLy = 2;
    private static final int URx = 3;
    private static final int URy = 4;
    private static final int SC = 0;
    private static final int TR = 4;

    // physical values for LLx, LLy, URx, URy
    private static double PLLx;
    private static double PLLy;
    private static double PURx;
    private static double PURy;

    private static final int CAN_COLOR = 0;

    private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
    private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    private static final TextLCD lcd = LocalEV3.get().getTextLCD();

    /** The tile's length. */
    public static final double TILE_SIZE = 30.48;
    public static final double WHEEL_RAD = 2.2;
    public static final double TRACK = 11.2;
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
    private static float[][] arrayColor;

    private static Odometer odometer;
    private static DifferentialLightSensor leftDifferentialLightSensor;
    private static DifferentialLightSensor rightDifferentialLightSensor;
    private static angleCorrection angleCorrection;
    private static MedianDistanceSensor medSensor;
    private static USAngleCorrector usAngleCorrector;
    private static AxesLocalizer axesLocalizer;
    private static IntersectionLocalizer intersectionLocalizer;
    private static SearchNavigator searchNavigator;
    private static CircleFollow circleFollow;
    private static ColourLightSensor colourLightSensor;
    private static URnavigator urNavigator;

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

        // set up back-right light sensor
        backRightLSPort = LocalEV3.get().getPort("S3");
        backRightLS = new EV3ColorSensor(backRightLSPort);
        backRightLSProvider = backRightLS.getMode("Red");
        backRightLSSample = new float[backRightLSProvider.sampleSize()];

        // set up side light sensor
        sideLSPort = LocalEV3.get().getPort("S4");
        sideLS = new EV3ColorSensor(sideLSPort);
        sideLSProvider = sideLS.getMode("RGB");
        sideLSSample = new float[sideLSProvider.sampleSize()];

        odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
        Thread odoThread = new Thread(odometer);
        odoThread.start();

        movementController = new MovementController(leftMotor, rightMotor, WHEEL_RAD, TRACK, odometer);

        leftDifferentialLightSensor = new DifferentialLightSensor(backLeftLSProvider, backLeftLSSample);
        rightDifferentialLightSensor = new DifferentialLightSensor(backRightLSProvider, backRightLSSample);

        angleCorrection = new angleCorrection(rightDifferentialLightSensor, leftDifferentialLightSensor,
                movementController, odometer);
        colourLightSensor = new ColourLightSensor(sideLSProvider, sideLSSample);
        medSensor = new MedianDistanceSensor(sideDistanceProvider, sideUSSample, odometer);
        urNavigator = new URnavigator(PURy, PURx, movementController, odometer);
        circleFollow = new CircleFollow(movementController, odometer, medSensor, colourLightSensor, TR, urNavigator,angleCorrection	);
        usAngleCorrector = new USAngleCorrector(movementController, odometer, medSensor);
        axesLocalizer = new AxesLocalizer(movementController, odometer, leftDifferentialLightSensor,
                rightDifferentialLightSensor);
        intersectionLocalizer = new IntersectionLocalizer(leftDifferentialLightSensor, movementController, odometer);
        searchNavigator = new SearchNavigator(odometer, movementController, LLx, LLy, URx, URy, medSensor, circleFollow,
                angleCorrection);
        llNavigator = new LLnavigator(SC, PLLx, PLLy, usAngleCorrector, intersectionLocalizer, axesLocalizer, movementController, odometer);

        do {
            /**
             * Clears the LCD and displays the main question: Do we want Rising Edge or Falling Edge?
             * On the left is RisingEdge method is needed, or on the right if FallingEdge method is required.
             */
            lcd.clear();

            lcd.drawString("left for routine", 0, 0);
            lcd.drawString("right for colour", 0, 1);
            
            buttonChoice = Button.waitForAnyPress();
        }
        while (buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_LEFT);
        System.out.println(buttonChoice);

        if (buttonChoice == Button.ID_LEFT) {
//            System.out.println("going up");
            lcd.clear();
            /**
             * usAngleCorrector.fallingEdge(); axesLocalizer.estimatePosition();
             * axesLocalizer.travelCloseToOrigin();
             * intersectionLocalizer.getIntersections();
             * intersectionLocalizer.correctAngle();
             * intersectionLocalizer.correctPosition(); movementController.travelTo(0, 0,
             * false); movementController.turnTo(0);
             **/

            llNavigator.navigateToLL();

            searchNavigator.searchPath();

            urNavigator.navigateToUr();

        }
        else if (buttonChoice == Button.ID_RIGHT) {
            medSensor.flush();
            searchNavigator.timedOut();
        }

//        System.out.println("Big wtf");
        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
        System.exit(0);
    }

}