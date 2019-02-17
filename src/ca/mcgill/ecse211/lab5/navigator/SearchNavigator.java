package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.localization.angleCorrection;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.UltrasonicMedianFilter;
import lejos.utility.TimerListener;

//takes in integer, not physical measures
public class SearchNavigator implements TimerListener{
	//
    private Odometer odometer;
    private MovementController movementController;
    private UltrasonicMedianFilter USdata;
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
                           int llX, int llY, int urX, int urY, UltrasonicMedianFilter USdata, 
                           wallFollower wallFollower, angleCorrection angleCorrector) 
    {
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
    	for(int n=deltaY, m=deltaX, i=0 ; n>0 & m>0 & i<10; n--, m--,i++) {
    		movementController.rotateAngle(90, false);
    		Ydistance = (n+1)*TILE_LENGTH;
    		movementController.driveDistance(Ydistance,true);
    		//poll USsensor somhow in for loop
    		movementController.rotateAngle(90, false);
    		Xdistance = (m+1)*TILE_LENGTH;
    		movementController.driveDistance(Xdistance,true);
    	
    		
    	}
   
    
    }

	@Override
	public void timedOut() {
		
		double canDist= USdata.getMedian();
		
		//if US sensor detects a can
		if (canDist<10){
		distanceLeft = (deltaX+0.5)-odometer.getXYT()[0];
		referencePos = odometer.getXYT();
		wallF.wallFollow();
		}
		//after it breaks from wallfollowing
		movementController.driveDistance(-TILE_LENGTH/2);
		movementController.driveDistance(2*TILE_LENGTH, true);
		angleCorrector.quickThetaCorrection();
		
	//	movementController.travelTo(odometer, referencePos[0], referencePos[1]);
		movementController.driveDistance(distanceLeft);
		
	} 
}
