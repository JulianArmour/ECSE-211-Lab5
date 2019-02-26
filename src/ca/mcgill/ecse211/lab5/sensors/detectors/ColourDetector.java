package ca.mcgill.ecse211.lab5.sensors.detectors;

import java.util.ArrayList;
import java.util.Collections;

/**
 * Class for color recognition of cans
 * @author Cedric
 *
 */
public class ColourDetector {

	private static final float RCAN_RMEAN = 0.8371f;
	private static final float RCAN_GMEAN = 0.4640f;
	private static final float RCAN_BMEAN = 0.2895f;
	
	private static final float BCAN_RMEAN = 0.5583f;
	private static final float BCAN_GMEAN = 0.6786f;
	private static final float BCAN_BMEAN = 0.4771f;
	
	private static final float YCAN_RMEAN = 0.8515f;
	private static final float YCAN_GMEAN = 0.5303f;
	private static final float YCAN_BMEAN = 0.2114f;
	
	private static final float GCAN_RMEAN = 0.6558f;
	private static final float GCAN_GMEAN = 0.6608f; 
	private static final float GCAN_BMEAN = 0.3649f;
	
	
	public static int verifyCan(float[][] data) {
		float RMean = 0.0f;
		float GMean = 0.0f;
		float BMean = 0.0f;
		float NRMean, NGMean, NBMean;
		
		for(int i = 0; i < data.length; i++) {
			RMean += data[i][0];
		}
		RMean /= data.length;
		for(int i = 0; i < data.length; i++) {
			GMean += data[i][1];
		}
		GMean /= data.length;
		for(int i = 0; i < data.length; i++) {
			BMean += data[i][2];
		}
		BMean /= data.length;
		
		NRMean = (float) (RMean / Math.sqrt(Math.pow(RMean, 2) + Math.pow(GMean, 2) + Math.pow(BMean, 2)));
		NGMean = (float) (GMean / Math.sqrt(Math.pow(RMean, 2) + Math.pow(GMean, 2) + Math.pow(BMean, 2)));
		NBMean = (float) (BMean / Math.sqrt(Math.pow(RMean, 2) + Math.pow(GMean, 2) + Math.pow(BMean, 2)));
		
//		System.out.println("NR: " + NRMean);
//		System.out.println("NG: " + NGMean);
//		System.out.println("NB: " + NBMean);
		
		return colorMatch(NRMean, NGMean, NBMean);
	}
	private static int colorMatch(float RMean, float GMean, float BMean) {
		Float dRCan, dBCan, dYCan, dGCan;
		float min;
		int dataCanColor = 0;
		ArrayList<Float> dArray = new ArrayList<Float>();
		
		dRCan = (float) Math.sqrt(Math.pow((RMean - RCAN_RMEAN), 2) + Math.pow((GMean - RCAN_GMEAN), 2) + Math.pow((BMean - RCAN_BMEAN), 2));
		dBCan = (float) Math.sqrt(Math.pow((RMean - BCAN_RMEAN), 2) + Math.pow((GMean - BCAN_GMEAN), 2) + Math.pow((BMean - BCAN_BMEAN), 2));
		dYCan = (float) Math.sqrt(Math.pow((RMean - YCAN_RMEAN), 2) + Math.pow((GMean - YCAN_GMEAN), 2) + Math.pow((BMean - YCAN_BMEAN), 2));
		dGCan = (float) Math.sqrt(Math.pow((RMean - GCAN_RMEAN), 2) + Math.pow((GMean - GCAN_GMEAN), 2) + Math.pow((BMean - GCAN_BMEAN), 2));
		
		dArray.add(dRCan);
		dArray.add(dBCan);
		dArray.add(dYCan);
		dArray.add(dGCan);
		
		min = Collections.min(dArray);
		
//		System.out.println("min: " + min);
		
		if(min == dRCan) dataCanColor = 4;
		else if(min == dYCan) dataCanColor = 3;
		else if(min == dGCan) dataCanColor = 2;
		else dataCanColor = 1;
		
		return dataCanColor;
		
		
		
	}
	
}
