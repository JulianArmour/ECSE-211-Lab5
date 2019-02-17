package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.Lab5;

//since this class generates waypoints, it takes in the coordinates not actual physical measures!!
public class waypointGenerator {
	private static int llx;
	private static int lly;
	private static int URx;
	private static int URy;
	private static double TILE_LENGTH = Lab5.TILE_SIZE;
	private static int deltaY;
	private static int deltaX;
	private double[][] searchPath;

	public waypointGenerator(int LLX, int LLY, int URX, int URY) {
		this.llx=LLX;
		this.lly=LLY;
		this.URx=URX;
		this.URy=URY;

	}

	public void generateWaypoints() {

		deltaY=(URy-lly);
		deltaX=(URx-llx);
		
		for(int n=deltaY, m=deltaX, i=0 ; n>0 & m>0 & i<10; n--, m--,i++) {
			
			//calculate x of next point
			searchPath[i][0]=(llx+m+0.5)
			//calculate y of next point
			//put both in array
			
			
			
			

		}

	}


}
