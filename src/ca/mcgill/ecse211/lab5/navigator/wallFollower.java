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
    private double[] odoData;
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

    public void wallFollow() {

        odoData = odo.getXYT(); // gets the initial XYT when wallfollower is started
        USdata.flush(); // reset filtered data

        while (odoData[2] - 5 <= odo.getXYT()[2] && odo.getXYT()[2] < odoData[2]) {

            
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
                // TODO Auto-generated catch block
                e.printStackTrace();

            }

        }

    }

}
