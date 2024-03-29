package ca.mcgill.ecse211.lab5.navigator;

import java.util.List;
import java.util.Iterator;
import java.util.LinkedList;

import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.detectors.ColourDetector;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.ColourLightSensor;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;
import lejos.hardware.Sound;
import lejos.robotics.ColorDetector;

public class WallFollower {

    private static int BAND_CENTER = 2; // (in cm)
    private static int BAND_WIDTH = 1; // (in cm)
    private static int MAX_DELTA = 150;
    private static double pFactor = 2;
    private static int MOTOR_SPEED = 120;
    
    private MovementController movementControler;
    private Odometer odo;
    private ColourLightSensor colorsensor;
  
    private double[] odoBeforeWallFollow;
    private double error;
    
    private MedianDistanceSensor USdata;
    private double distance;
    private int TARGET_COLOR;
    
    private List<float[]> LTdata;
	private float[][] colourdata;

    // constructor for wallFollower class
    public WallFollower(MovementController movementCtr, Odometer odometer, MedianDistanceSensor USfilter,
    		ColourLightSensor colorsensor, int TARGET_COLOR) {

        this.movementControler = movementCtr;
        this.odo = odometer;
        this.USdata = USfilter;
        this.colorsensor = colorsensor;
        
        this.LTdata = new LinkedList<float[]>();
        this.TARGET_COLOR = TARGET_COLOR;
        		
    }

    /**
     * Moves the robot around the can and polls colour samples, then returns 
     * the robot to it's original position before going around the can
     */
    public void wallFollow() {

        odoBeforeWallFollow = odo.getXYT(); // gets the initial XYT when wallfollower is started
        
//        USdata.flush(); // reset filtered data

        //  start collecting colour data
        
        
        // do 7/8th of a circle around the can
        //((theta - heading + 360) % 360)
        double breakOutAngle = odoBeforeWallFollow[2] + 40.0;
        while (Math.abs(odo.getXYT()[2] - breakOutAngle) > 20) {
            
        	// polls the ColorSensor and puts it in an array
        	float[] colorData = colorsensor.fetchColorSamples();
        	
        	if (colorData[0] > 0.001 || colorData[1] > 0.001 || colorData[2] > 0.001) {
                LTdata.add(colorData);
        	}
        	
        	//polls the ultrasonic distance
            distance = USdata.getFilteredDistance();

            
            //start of actual wallfollowing code
            error = BAND_CENTER - distance;

            // out of bounds
            if (Math.abs(error) > BAND_WIDTH / 2) {

                int scaledDelta = (int) (Math.abs(error * pFactor));

                if (scaledDelta > MAX_DELTA) {
                    scaledDelta = MAX_DELTA;
                }

                // this.scaledSpeed = scaledDelta; why is dis here?

                // too close to the wall
                if (error >= 0) {
                    movementControler.turnRight(MOTOR_SPEED, scaledDelta);
                }
                // too far away from the wall
                else {
                    movementControler.turnLeft(MOTOR_SPEED, scaledDelta);
                }
            } else {
                movementControler.driveForward();
            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();

            }

        }
        // make the robot go back to it's original position and orientation
        movementControler.travelTo(odoBeforeWallFollow[0], odoBeforeWallFollow[1], false);
        movementControler.turnTo(odoBeforeWallFollow[2]);
//        colourdata =  (float[][]) LTdata.toArray();
        colourdata = new float[LTdata.size()][3];
        int i = 0;
        for (Iterator<float[]> iterator = LTdata.iterator(); iterator.hasNext();) {
            colourdata[i] = (float[]) iterator.next();
            i++;
        }
        /**
        if(ColourDetector.verifyCan(colourdata, TARGET_COLOR)) {
            //beep once
            Sound.beep();
        }
        else {
            //beep twice
            Sound.twoBeeps();
        }**/
    }

}
