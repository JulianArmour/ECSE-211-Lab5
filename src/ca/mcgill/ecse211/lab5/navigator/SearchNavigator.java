package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.localization.angleCorrection;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;
import lejos.utility.TimerListener;

//takes in integer, not physical measures
public class SearchNavigator implements TimerListener {

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
    }

    public void searchPath() {

        // start an angle correction thread

        deltaY = (int) ((urY - llY) / TILE_LENGTH);
        deltaX = (int) ((urX - llX) / TILE_LENGTH);

        movementController.driveDistance(-TILE_LENGTH / 2);
        movementController.turnTo(90);
        // hardcoded part on x axis

        Xdistance = deltaX + 0.5;
        movementController.driveDistance(Xdistance);

        // for loop for remaning path
        for (int i = 0; deltaX > 0 & deltaY > 0 & i < 10; deltaX--, deltaY--, i++) {
            movementController.rotateAngle(90, false);
            Ydistance = (deltaX + 1) * TILE_LENGTH;
            movementController.driveDistance(Ydistance, true);
            movementController.rotateAngle(90, false);
            Xdistance = (deltaY + 1) * TILE_LENGTH;
            movementController.driveDistance(Xdistance, true);
        }

    }

    @Override
    public void timedOut() {

        double canDist = USdata.getFilteredDistance();

        // if US sensor detects a can
        if (canDist < 10) {

            canDetected = true; // maybe use this to influence the for loop to interrupt
            // set up things before going into wallfollowing mode
            // if robot is moving in x-axis
            if (movementController.roundAngle(odometer) == 90 || movementController.roundAngle(odometer) == 270) {
                distanceLeft = (Xdistance) - odometer.getXYT()[0];
            }
            // if robot is moving in y-axis
            if (movementController.roundAngle(odometer) == 0 || movementController.roundAngle(odometer) == 180) {
                distanceLeft = (Ydistance) - odometer.getXYT()[1];
            }
            referencePos = odometer.getXYT();

            // goes into wallfollowing mode
            wallF.wallFollow();
        }
        // NEED TO DO CALCULATIONS HERE!!!!!!!

        // after it breaks from wallfollowing
        // movementController.driveDistance(-TILE_LENGTH/2); might not need it because
        // sensor in the back
        movementController.driveDistance(2 * TILE_LENGTH, true);
        angleCorrector.quickThetaCorrection();

        // movementController.travelTo(odometer, referencePos[0], referencePos[1]);
        movementController.driveDistance(distanceLeft);

    }
}
