package ca.mcgill.ecse211.lab5.localization;

import ca.mcgill.ecse211.lab5.navigator.MovementController;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;

/**
 * Provides methods for finding what angle the robot thinks it's facing
 * and correcting it to the actual angle it faces.
 * 
 * @author Julian Armour
 * @since 11-02-2019
 * @version 1.03
 */
public class USAngleCorrector {
    // a difference past this value consitues a falling or rising edge
    private static final double EDGE_THRESHOLD = 55.0;
//    private static final double EDGE_LIMIT = 250.0;
    // period between distance samples
    private static final long US_POLL_PERIOD = 300;
    private static final int MAX_DIST = 255;
    private MovementController movementController;
    private MedianDistanceSensor med;
    private Odometer odometer;

    /**
     * Creates a USAngleCOrrector object
     * 
     * @param movementController            The {@link MovementController}
     * @param differentialDistancePoller    The {@link DifferentialDistancePoller}
     * @param odometer                      The {@link Odometer}
     */
    public USAngleCorrector(MovementController movementController, Odometer odometer, MedianDistanceSensor medSensor) {
        this.movementController = movementController;
        this.odometer = odometer;
        this.med = medSensor;
    }
    
    /**
     * The subroutine for correcting the odometer's angle. If the robot is placed at a corner tile
     * with walls on each side of the corner, then the robot will "scan" while rotating. It is looking
     * for a large drop in distance measured by the {@link DifferentialDistancePoller}. It will record
     * at what angles these large differences in distance occured at and use them to calculate the 
     * {@link Odometer}'s angle error.
     */
    public void fallingEdge() {
        double fallingEdge = 404; // 404: the angle has not been found
        double risingEdge = 404;
        boolean rotatingClockwise = true; // to keep track of what direction the robot is rotating
        boolean firstFallingEdgeOnBackWall = false; // to know if the first falling edge was on the back wall
        

        /* The median filter needs to get some data to prevent a spike at the beginning since it
         * is constructed with dummy values. So it is loaded with 10 samples.
         */
        for (int i = 0; i < 10; i++) {
            med.getFilteredDistance();
            try {
                Thread.sleep(US_POLL_PERIOD);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        
        // make the robot rotate counter-clockwise until it sees a large change in distance
        movementController.rotateAngle(720, false, true);
        // move out of valley
        while (Math.min(med.getFilteredDistance(), MAX_DIST) < 55) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        
        // then start moving clockwise
        movementController.rotateAngle(720, true, true);
        
        int dist;
        while ((dist = Math.min(med.getFilteredDistance(), MAX_DIST)) > EDGE_THRESHOLD) {
         // let some time pass between each sample
            try {
                Thread.sleep(US_POLL_PERIOD);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        fallingEdge = odometer.getXYT()[2];
        System.out.println("Found falling edge, dist =  "+dist);
        
        while ((dist = Math.min(med.getFilteredDistance(), MAX_DIST)) < EDGE_THRESHOLD) {
            // let some time pass between each sample
               try {
                   Thread.sleep(US_POLL_PERIOD);
               } catch (InterruptedException e) {
                   e.printStackTrace();
               }
           }
        risingEdge = odometer.getXYT()[2];
        System.out.println("Found rising edge, dist =  "+dist);
        
        System.out.println("first falling edge: "+fallingEdge);
        System.out.println("second falling edge: "+risingEdge);
        System.out.println("First was backwall? "+firstFallingEdgeOnBackWall);
        
        double alpha = fallingEdge;
        double beta = risingEdge;
        
        // the odometer's error, to be added to the current odometer's angle later
        double dTheta;
        
        if (alpha <= beta) {
            dTheta = 345 - (alpha + beta) / 2;
            System.out.println("alpha <= beta");
        } else {
            dTheta = 155 - (alpha + beta) / 2; // increase in ccw direction
            System.out.println("alpha > beta");
        }
        
        /* correct the odometer. Note: the motor's didn't need to be stopped to perform the update
         * since the odometer is thread-safe.
         */
        odometer.update(0, 0, dTheta);
        // face "North"
        movementController.turnTo(0.0);
    }
}
