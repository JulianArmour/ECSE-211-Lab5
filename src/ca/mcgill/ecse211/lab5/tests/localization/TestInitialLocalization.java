package ca.mcgill.ecse211.lab5.tests.localization;

import ca.mcgill.ecse211.lab5.localization.AxesLocalizer;
import ca.mcgill.ecse211.lab5.localization.IntersectionLocalizer;
import ca.mcgill.ecse211.lab5.localization.USAngleCorrector;
import ca.mcgill.ecse211.lab5.localization.angleCorrection;
import ca.mcgill.ecse211.lab5.navigator.LLnavigator;
import ca.mcgill.ecse211.lab5.navigator.MovementController;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.odometer.OdometerExceptions;
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

public class TestInitialLocalization {
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

    private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
    private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    private static final TextLCD lcd = LocalEV3.get().getTextLCD();

    /** The tile's length. */
    public static final double TILE_SIZE = 30.48;
    public static final double WHEEL_RAD = 2.1;
    public static final double TRACK = 11.8;
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
    private static DifferentialLightSensor leftDifferentialLightSensor;
    private static DifferentialLightSensor rightDifferentialLightSensor;
    private static angleCorrection angleCorrection;
    private static MedianDistanceSensor medSensor;
    private static USAngleCorrector usAngleCorrector;
    private static AxesLocalizer axesLocalizer;
    private static IntersectionLocalizer intersectionLocalizer;

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
        
//        // set up side light sensor
//        sideLSPort = LocalEV3.get().getPort("S4");
//        sideLS= new EV3ColorSensor(sideLSPort);
//        sideLSProvider = sideLS.getMode("RGB");
//        sideLSSample = new float[sideLSProvider.sampleSize()];
        
        
        odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
        (new Thread(odometer)).start();
        
        movementController = new MovementController(leftMotor, rightMotor, WHEEL_RAD, TRACK, odometer);
        
        leftDifferentialLightSensor = new DifferentialLightSensor(backLeftLSProvider, backLeftLSSample);
        rightDifferentialLightSensor = new DifferentialLightSensor(backRightLSProvider, backRightLSSample);
        
        angleCorrection = new angleCorrection(rightDifferentialLightSensor, leftDifferentialLightSensor,
                                              movementController, odometer);
        medSensor = new MedianDistanceSensor(sideDistanceProvider, sideUSSample, odometer);
        usAngleCorrector = new USAngleCorrector(movementController, odometer, medSensor);
        axesLocalizer = new AxesLocalizer(movementController, odometer, leftDifferentialLightSensor, rightDifferentialLightSensor);
        intersectionLocalizer = new IntersectionLocalizer(leftDifferentialLightSensor, movementController, odometer);
        
        do {
            lcd.clear();
            lcd.drawString("Press ^UP^ to start", 0, 0);
            buttonChoice = Button.waitForAnyPress();
        } while (buttonChoice != Button.ID_UP);
        
        
        if (buttonChoice == Button.ID_UP) {
            while (buttonChoice != Button.ID_ESCAPE) {
                usAngleCorrector.fallingEdge();
                axesLocalizer.estimatePosition();
                axesLocalizer.travelCloseToOrigin();
                intersectionLocalizer.getIntersections();
                intersectionLocalizer.correctAngle();
                intersectionLocalizer.correctPosition();
                movementController.travelTo(0, 0, false);
                movementController.turnTo(0);
                
                buttonChoice = Button.waitForAnyPress();
            }
        }

        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
        System.exit(0);
    }

}
