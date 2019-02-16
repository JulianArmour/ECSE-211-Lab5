package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.localization.AxesLocalizer;
import ca.mcgill.ecse211.lab5.localization.IntersectionLocalizer;
import ca.mcgill.ecse211.lab5.localization.LightLocalisation;
import ca.mcgill.ecse211.lab5.localization.USLocalisation;
import ca.mcgill.ecse211.lab5.odometer.Odometer;

public class LLnavigator {

	private static int StartC;
	private static double LLx;
	private static double LLy;
	private USLocalisation usLocalizer;
	private IntersectionLocalizer interLocalizer;
	private AxesLocalizer axesLocalizer;
	private MovementController movCon;
	private Odometer odo;
	

	public LLnavigator(int SC, double LLx, double LLy, USLocalisation usLocalizer,
			IntersectionLocalizer interLocalizer, AxesLocalizer axesLocalizer, 
			MovementController movCon,Odometer odo) {

		this.StartC = SC;
		this.LLx= LLx;
		this.LLy=LLy;
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
			
			//navigate to LLx,LLy
			movCon.travelTo(odo, LLx, LLy);
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
			
			odo.setXYT(7*TILE_SIZE, 1*TILE_SIZE, 270);
			
			
			//move to 0
			movCon.travelTo(odo, 10,10);
		

		}

		else if(StartC == 2) {


		}
		else if (StartC == 3) {
			
		}

		
	}
}
