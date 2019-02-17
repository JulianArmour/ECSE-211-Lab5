package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;

public class wallFollower {

    private static int BAND_CENTER = 5; // (in cm)
    private static int BAND_WIDTH = 1; // (in cm)
    private static int MAX_DELTA = 50;
    private static double pFactor = 1.5;
    private MovementController movementControler;
    private Odometer odo;
    private double[] odoBeforeWallFollow;
    private double error;
    private static int MOTOR_SPEED = 100;
    private MedianDistanceSensor USdata;
    private double distance;

    // constructor for wallFollower class
    public wallFollower(MovementController movementCtr, Odometer odometer, MedianDistanceSensor USfilter) {

        this.movementControler = movementCtr;
        this.odo = odometer;
        this.USdata = USfilter;
    }

    /**
     * Moves the robot around the can and polls colour samples, then returns 
     * the robot to it's original position before going around the can
     */
    public void wallFollow() {

        odoBeforeWallFollow = odo.getXYT(); // gets the initial XYT when wallfollower is started
//        USdata.flush(); // reset filtered data

        // TODO start collecting colour data
        
        // do 7/8th of a circle around the can
        while (odoBeforeWallFollow[2] + 40 <= odo.getXYT()[2] && odo.getXYT()[2] < odoBeforeWallFollow[2] + 45) {
            
            distance = USdata.getFilteredDistance();

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
                    movementControler.turnRight(MOTOR_SPEED, scaledDelta + 10);
                }
                // too far away from the wall
                else {
                    movementControler.turnLeft(MOTOR_SPEED, scaledDelta);
                }
            } else {
                movementControler.driveForward();
            }

            try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                e.printStackTrace();

            }

        }
        // TODO make the robot go back to it's original position and orientation
    }

}
