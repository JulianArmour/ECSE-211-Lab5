package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.localization.AxesLocalizer;
import ca.mcgill.ecse211.lab5.localization.IntersectionLocalizer;
import ca.mcgill.ecse211.lab5.localization.USLocalisation;
import ca.mcgill.ecse211.lab5.odometer.Odometer;

public class LLnavigator {

	private static int StartC;
	private static double PLLX;
	private static double PLLY;
	private USLocalisation usLocalizer;
	private IntersectionLocalizer interLocalizer;
	private AxesLocalizer axesLocalizer;
	private MovementController movCon;
	private Odometer odo;
	

	public LLnavigator(int SC, double PPLLX, double PPLLY, USLocalisation usLocalizer,
			IntersectionLocalizer interLocalizer, AxesLocalizer axesLocalizer, 
			MovementController movCon,Odometer odo) {

		this.StartC = SC;
		this.PLLX= PPLLX;
		this.PLLY=PPLLY;
		this.usLocalizer = usLocalizer;
		this.interLocalizer = interLocalizer;
		this.axesLocalizer=axesLocalizer;
		this.movCon=movCon;
		this.odo=odo;
		
	}

	public void navigateToLL() {

		if (StartC == 0) {
			//us localize
			usLocalizer.run();
			
			
			//light localize (2 steps)
			//NEED TO SET X AND Y AXES
			axesLocalizer.estimatePosition(); //sets the X and Y approx.
			interLocalizer.getIntersections();	//finds the 4 axes
			interLocalizer.correctAngle();		//corrects theta
			interLocalizer.correctPosition();	//corrects X and Y pos.
			
			//navigate to PLLX,PLLY
			movCon.travelTo(PLLX, PLLY);
		}

		else if (StartC == 1) {
			//us localize
			usLocalizer.run();
			//light localize (2 steps)
			//NEED TO SET X AND Y AXES
			axesLocalizer.estimatePosition(); //sets the X and Y approx.
			interLocalizer.getIntersections();	//finds the 4 axes
			interLocalizer.correctAngle();		//corrects theta
			interLocalizer.correctPosition();	//corrects X and Y pos.
			
			
			odo.setXYT(7*Lab5.TILE_SIZE, 1*Lab5.TILE_SIZE, 270);
			
			//move to 0
			movCon.rotateAngle(90, false);
			movCon.driveDistance(Lab5.TILE_SIZE/2);
			movCon.travelTo(Lab5.TILE_SIZE-5,Lab5.TILE_SIZE-5);
			
			//ITS IN CORNER 0
			interLocalizer.getIntersections();	//finds the 4 axes
			interLocalizer.correctAngle();		//corrects theta
			interLocalizer.correctPosition();	//corrects X and Y pos.
			
			odo.setXYT(Lab5.TILE_SIZE, Lab5.TILE_SIZE, 0);
			
			movCon.travelTo(PLLX, PLLY);
			
			

		}

		else if(StartC == 2) {
			//us localize
			usLocalizer.run();
			//light localize (2 steps)
			//NEED TO SET X AND Y AXES
			axesLocalizer.estimatePosition(); //sets the X and Y approx.
			interLocalizer.getIntersections();	//finds the 4 axes
			interLocalizer.correctAngle();		//corrects theta
			interLocalizer.correctPosition();	//corrects X and Y pos.
			
			
			odo.setXYT(7*Lab5.TILE_SIZE, 7*Lab5.TILE_SIZE, 180);
			
			movCon.rotateAngle(90, false);
			movCon.driveDistance(Lab5.TILE_SIZE/2);
			movCon.travelTo(Lab5.TILE_SIZE/2,Lab5.TILE_SIZE/2);
			movCon.travelTo(Lab5.TILE_SIZE-5,Lab5.TILE_SIZE-5);
			
			//ITS IN CORNER 0
			interLocalizer.getIntersections();	//finds the 4 axes
			interLocalizer.correctAngle();		//corrects theta
			interLocalizer.correctPosition();	//corrects X and Y pos.
			
			odo.setXYT(Lab5.TILE_SIZE, Lab5.TILE_SIZE, 0);
			
			movCon.travelTo(PLLX, PLLY);
			
			


		}
		else if (StartC == 3) {
			//us localize
			usLocalizer.run();
			//light localize (2 steps)
			//NEED TO SET X AND Y AXES
			axesLocalizer.estimatePosition(); //sets the X and Y approx.
			interLocalizer.getIntersections();	//finds the 4 axes
			interLocalizer.correctAngle();		//corrects theta
			interLocalizer.correctPosition();	//corrects X and Y pos.
			
			
			odo.setXYT(1*Lab5.TILE_SIZE, 7*Lab5.TILE_SIZE, 180);
			
			//move to 0
			movCon.rotateAngle(90, true);
			movCon.driveDistance(Lab5.TILE_SIZE/2);
			movCon.travelTo(Lab5.TILE_SIZE-5,Lab5.TILE_SIZE-5);
			
			//ITS IN CORNER 0
			interLocalizer.getIntersections();	//finds the 4 axes
			interLocalizer.correctAngle();		//corrects theta
			interLocalizer.correctPosition();	//corrects X and Y pos.
			
			odo.setXYT(Lab5.TILE_SIZE, Lab5.TILE_SIZE, 0);
			
			movCon.travelTo(PLLX, PLLY);
			
		}

		
	}
}
