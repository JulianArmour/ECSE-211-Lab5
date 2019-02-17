package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor.MedianDistanceSensor;

//takes in integer, not physical measures
public class SearchNavigator {
	//
    private Odometer odometer;
    private MovementController movementController;
    private MedianDistanceSensor USdata;
    private wallFollower wallF;
    private int llX;
    private int llY;
    private int urX;
    private int urY;
    private double TILE_LENGTH= Lab5.TILE_SIZE;
    private int deltaY;
    private int deltaX;
    private double distanceLeft;
    private double canDist;
    

    public SearchNavigator(Odometer odometer, MovementController movementController, 
                           int llX, int llY, int urX, int urY, MedianDistanceSensor USdata, wallFollower wallFollower) 
    {
        this.odometer = odometer;
        this.movementController = movementController;
        this.USdata=USdata;
        this.llX = llX;
        this.llY = llY;
        this.urX = urX;
        this.urY = urY;
        this.wallF=wallFollower;
    }
    
    private void searchPath() {
    	
    	//start an angle correction thread
    	
    	deltaY=(int) ((urY-llY)/TILE_LENGTH);
		deltaX=(int) ((urX-llX)/TILE_LENGTH);
		
		movementController.driveDistance(-TILE_LENGTH/2);
		movementController.turnTo(90);
		//hardcoded part on x axis
		
		movementController.driveDistance(deltaX+0.5);
		
		while(true) {
			
			canDist = USdata.getFilteredDistance();
			if(canDist<10) break;
		}
		//if US sensor detects a can
		
			distanceLeft = (deltaX+0.5)-odometer.getXYT()[0];
			wallF.wallFollow();
			movementController.driveDistance(distanceLeft);
		
			
		
		//for loop for remaning path
    	for(int n=deltaY, m=deltaX, i=0 ; n>0 & m>0 & i<10; n--, m--,i++) {
    		movementController.rotateAngle(90, false);
    		movementController.driveDistance((n+1)*TILE_LENGTH,true);
    		//poll USsensor somhow in for loop
    		movementController.rotateAngle(90, false);
    		movementController.driveDistance((m+1)*TILE_LENGTH,true);
    	
    		
    	}
    
    
    } 
}
