package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.localization.angleCorrection;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

//takes in integer, not physical measures
public class SearchNavigator implements TimerListener {



    private static final double DESTINATION_THRESHOLD = 2.0;
    private static final int CAN_SCAN_PERIOD = 100;
    private Odometer odometer;
    private MovementController movementController;
    private MedianDistanceSensor USdata;
    private wallFollower wallF;
    private angleCorrection angleCorrector;
    private int llX;
    private int llY;
    private int urX;
    private int urY;
    private double TILE_LENGTH = Lab5.TILE_SIZE;
    private int deltaY;
    private int deltaX;
    private double distanceLeft;
    private double canDist;
    private double[] referencePos;
    private double Xdistance;
    private double Ydistance;
    boolean canDetected = false;
    private double[] destination;
    double[] currentPos;
    Timer timer;

    public SearchNavigator(Odometer odometer, MovementController movementController, int llX, int llY, int urX, int urY,
            MedianDistanceSensor USdata, wallFollower wallFollower, angleCorrection angleCorrector) {

        this.odometer = odometer;
        this.movementController = movementController;
        this.USdata = USdata;
        this.angleCorrector = angleCorrector;
        this.llX = llX;
        this.llY = llY;
        this.urX = urX;
        this.urY = urY;
        this.wallF = wallFollower;
        this.currentPos = new double[3];
    }

    public void searchPath() {

        // start an angle correction thread

        deltaY = (int) (urY - llY);
        deltaX = (int) (urX - llX);

        movementController.driveDistance(-TILE_LENGTH / 2);
        movementController.turnTo(90);
        // hardcoded part on x axis

        //start the timer
        timer = new Timer(CAN_SCAN_PERIOD, this);
        timer.start();
        
        Xdistance = (deltaX + 0.5)*TILE_LENGTH;
        currentPos = odometer.getXYT();
        destination = new double[] {currentPos[0] + Xdistance, currentPos[1], currentPos[2] };
        movementController.travelTo(destination[0], destination[1],true);
        
//        movementController.driveDistance(Xdistance);
        // TODO check for cans while driving
        
        
        
        

        // for loop of remaning path
        for (int n = deltaX, m = deltaY, i = 0; n > 0 & m > 0 & i < 10; n--, m--, i++) {
            movementController.rotateAngle(90, false);
            // perform a quick angle correction
            angleCorrector.quickThetaCorrection();
            
         // start traveling
            Ydistance = (n + 1) * TILE_LENGTH;
            
            if(movementController.roundAngle()==0) {
	            double[] currentPos = odometer.getXYT();
	            destination[0] = currentPos[0];
	            destination[1] = currentPos[1]+Ydistance;
	            destination[2] = currentPos[2];
            }
            else if(movementController.roundAngle() == 180) {
            	double[] currentPos = odometer.getXYT();
	            destination[0] = currentPos[0];
	            destination[1] = currentPos[1]-Ydistance;
	            destination[2] = currentPos[2];
            }
            
            movementController.travelTo(destination[0], destination[1],true);
            movementController.travelTo(destination[0], destination[1], true);
            // TODO check for cans
            
            while (distanceToDestination() > DESTINATION_THRESHOLD) {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            timer.stop();
            movementController.travelTo(destination[0], destination[1], false);
            
            // TODO move to next navigation after reaching destination
            
            movementController.rotateAngle(90, false);
            angleCorrector.quickThetaCorrection();
            
            Xdistance = (m + 1) * TILE_LENGTH;
            
            if(movementController.roundAngle() == 90) {
            	double [] currentPos = odometer.getXYT();
            	destination[0]=currentPos[0] + Xdistance;
            	destination[1]=currentPos[1];
            	destination[2]= currentPos[2];
           }
            else if (movementController.roundAngle() == 270) {
            	double [] currentPos = odometer.getXYT();
            	destination[0]=currentPos[0] - Xdistance;
            	destination[1]=currentPos[1];
            	destination[2]= currentPos[2];
            }
            movementController.travelTo(destination[0], destination[1],true);
            // TODO check for cans
        }
//
    }

    private double distanceToDestination() {
        double[] curPos = odometer.getXYT();
        double dX = destination[0] - curPos[0];
        double dY = destination[1] - curPos[1];
        return Math.sqrt(dX * dX + dY * dY);
    }

    @Override
    public void timedOut() {

        double canDist = USdata.getFilteredDistance();

        // if US sensor detects a can
        if (canDist < 10) {
            
            movementController.stopMotors();

            canDetected = true; // maybe use this to influence the for loop to interrupt
            
            
            // goes into wallfollowing mode and collects colour data
            wallF.wallFollow();
            // Note: at this point the robot is back to where it was before wall-following
            
            // angle correction
            angleCorrector.quickThetaCorrection();
            
            //TODO keep moving remaining distance
            // if robot is moving in x-axis
            if (movementController.roundAngle() == 90 || movementController.roundAngle() == 270) {
                distanceLeft = (Xdistance) - odometer.getXYT()[0]; 
                /*
                 *  
                 *  of a distance remaining. distance = pos_destination - pos_current
                 */
            }
            // if robot is moving in y-axis
            else if (movementController.roundAngle() == 0 || movementController.roundAngle() == 180) {
                distanceLeft = (Ydistance) - odometer.getXYT()[1];
            }
        }
        // TODO NEED TO DO CALCULATIONS HERE!!!!!!!

        // after it breaks from wallfollowing
        // movementController.driveDistance(-TILE_LENGTH/2); might not need it because
        // sensor in the back
//        movementController.driveDistance(2 * TILE_LENGTH, true); // why drive 2 tiles?
        angleCorrector.quickThetaCorrection();

        // movementController.travelTo(odometer, referencePos[0], referencePos[1]);
        movementController.driveDistance(distanceLeft);

    }
}
