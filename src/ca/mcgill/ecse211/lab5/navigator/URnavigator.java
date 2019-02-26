package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.odometer.Odometer;

/**
 * provides methods for moving the robot to the upper right of the search zone
 * @author Alice Kazarine
 *
 */
public class URnavigator {
	//
	private MovementController movCon;
	private Odometer odo;

	private static double urY;
	private static double urX;
	private double [] currentPos;
	private double TILE_LENGTH = Lab5.TILE_SIZE;

	public URnavigator(double URy, double URx, MovementController movementController, Odometer odometer) {
		this.urX=URx;
		this.urY=URy;
		this.movCon=movementController;
		this.odo=odometer;

	}

	/**
	 * moves the robot to the upper right of the search zone.
	 */
	public void navigateToUr() {
		currentPos=odo.getXYT();

		if(movCon.roundAngle() == 0) {
			movCon.driveDistance(urY-currentPos[1]+TILE_LENGTH*1.5, false);
			movCon.travelTo(urX, urY, false);
		}
		else if(movCon.roundAngle() == 90) {
			movCon.driveDistance(Lab5.TILE_SIZE/2); //just to be in the middle of tile
			movCon.rotateAngle(90, false);
			 movCon.driveDistance(urY-currentPos[1]+TILE_LENGTH*1.5, false);
			movCon.travelTo(urX, urY, false);
		}
		else if(movCon.roundAngle() == 180) {

			movCon.rotateAngle(180, false);
			 movCon.driveDistance(urY-currentPos[1]+TILE_LENGTH*1.5, false);
			movCon.travelTo(urX, urY,false);

		}
		else if(movCon.roundAngle() == 270) {
			movCon.driveDistance(Lab5.TILE_SIZE/2); //just to be in the middle of tile
			movCon.rotateAngle(90, true);
			 movCon.driveDistance(urY-currentPos[1]+TILE_LENGTH*1.5, false);
			movCon.travelTo(urX, urY,false);

		}
		System.exit(0);
	}
}
