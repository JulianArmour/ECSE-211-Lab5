package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.localization.angleCorrection;
import ca.mcgill.ecse211.lab5.odometer.Odometer;

import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;
import lejos.utility.TimerListener;

//takes in integer, not physical measures
public class SearchNavigator implements TimerListener{
	
    private Odometer odometer;
    private MovementController movementController;
    private MedianDistanceSensor USdata;
    private wallFollower wallF;
    private angleCorrection angleCorrector;
    private int llX;
    private int llY;
    private int urX;
    private int urY;
    private double TILE_LENGTH= Lab5.TILE_SIZE;
    private int deltaY;
    private int deltaX;
    private double distanceLeft;
    private double canDist;
    private double[] referencePos;
    private double Xdistance;
    private double Ydistance;
    

    public SearchNavigator(Odometer odometer, MovementController movementController, 
                           int llX, int llY, int urX, int urY, MedianDistanceSensor USdata, 
                           wallFollower wallFollower, angleCorrection angleCorrector) {

        this.odometer = odometer;
        this.movementController = movementController;
        this.USdata=USdata;
        this.angleCorrector=angleCorrector;
        this.llX = llX;
        this.llY = llY;
        this.urX = urX;
        this.urY = urY;
        this.wallF=wallFollower;
    }
    
    public void searchPath() {
    	
    	//start an angle correction thread
    	
    	deltaY=(int) ((urY-llY)/TILE_LENGTH);
		deltaX=(int) ((urX-llX)/TILE_LENGTH);
		
		movementController.driveDistance(-TILE_LENGTH/2);
		movementController.turnTo(90);
		//hardcoded part on x axis
		
		Xdistance = deltaX+ 0.5;
		movementController.driveDistance(Xdistance);
		
		//for loop for remaning path
    	for(int i = 0; deltaX > 0 & deltaY > 0 & i < 10; deltaX--, deltaY--,i++) {
    		movementController.rotateAngle(90, false);
    		Ydistance = (deltaX+1)*TILE_LENGTH;
    		movementController.driveDistance(Ydistance,true);
    		//poll USsensor somhow in for loop
    		movementController.rotateAngle(90, false);
    		Xdistance = (deltaY+1)*TILE_LENGTH;
    		movementController.driveDistance(Xdistance,true);
    	
    		
    	}
   
    
    }

	@Override
	public void timedOut() {
		
		double canDist = USdata.getFilteredDistance();
		
		//if US sensor detects a can
		if (canDist<10){
    		/* TODO deltaX needs to be updated in the for loop since since it changes
    		 * with each iteration
    		 */		 
    		distanceLeft = (deltaX+0.5)-odometer.getXYT()[0];
    		referencePos = odometer.getXYT();
    		wallF.wallFollow();
    		//after it breaks from wallfollowing (3/4 of a circle around the can)
    		movementController.turnTo(angleSnap(odometer.getXYT()[2]));
    		movementController.driveDistance(TILE_LENGTH/2);
    		movementController.rotateAngle(90, false);
    		angleCorrector.quickThetaCorrection();
		}
	} 
	
	private static double angleSnap(double angle) {
	    return (Math.round(angle / 90.0) * 90) % 360;
	}
}
