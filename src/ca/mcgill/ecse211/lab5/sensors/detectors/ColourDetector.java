package ca.mcgill.ecse211.lab5.sensors.detectors;

public class ColourDetector {
	private static final int N_SD = 2;

	private static final float RCAN_RMEAN = 0.0116f;
	private static final float RCAN_RSD = 0.0030f;
	private static final float RCAN_GMEAN = 0.0048f;
	private static final float RCAN_GSD = 0.0018f;
	private static final float RCAN_BMEAN = 0.0035f;
	private static final float RCAN_BSD = 0.0013f;
	
	private static final float BCAN_RMEAN = 0.0063f;
	private static final float BCAN_RSD = 0.0023f;
	private static final float BCAN_GMEAN = 0.0073f;
	private static final float BCAN_GSD = 0.0017f;
	private static final float BCAN_BMEAN = 0.0094f;
	private static final float BCAN_BSD = 0.0014f;
	
	private static final float YCAN_RMEAN = 0.013f;
	private static final float YCAN_RSD = 0.0021f;
	private static final float YCAN_GMEAN = 0.0091f;
	private static final float YCAN_GSD = 0.0021f;
	private static final float YCAN_BMEAN = 0.0060f;
	private static final float YCAN_BSD = 0.0025f;
	
	private static final float GCAN_RMEAN = 0.0069f;
	private static final float GCAN_RSD = 0.0030f;
	private static final float GCAN_GMEAN = 0.0086f;
	private static final float GCAN_GSD = 0.0019f;
	private static final float GCAN_BMEAN = 0.006f;
	private static final float GCAN_BSD = 0.0024f;
	
	public static boolean verifyCan(float[][] data, int canColor) {
		float RMean = 0.0f;
		float GMean = 0.0f;
		float BMean = 0.0f;
		
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
		return colorMatch(RMean, GMean, BMean, chooseCanColor(canColor));
	}
	
	private static boolean colorMatch(float RMean, float GMean, float BMean, float[] canColor) {
		if(RMean >= (canColor[0]-N_SD*canColor[1]) && RMean <= (canColor[0]+N_SD*canColor[1])) {
			if(GMean >= (canColor[2]-N_SD*canColor[3]) && GMean <= (canColor[3]+N_SD*canColor[3])) {
				if(BMean >= (canColor[4]-N_SD*canColor[5]) && BMean <= (canColor[4]+N_SD*canColor[5])) {
					return true;
				}
			}
		}
		return false;
	}
	//method
	private static float[] chooseCanColor(int canColor) {
		float[] redCan = {RCAN_RMEAN, RCAN_RSD, RCAN_GMEAN, RCAN_GSD, RCAN_BMEAN, RCAN_BSD};
		float[] blueCan = {BCAN_RMEAN, BCAN_RSD, BCAN_GMEAN, BCAN_GSD, BCAN_BMEAN, BCAN_BSD};
		float[] yellowCan = {YCAN_RMEAN, YCAN_RSD, YCAN_GMEAN, YCAN_GSD, YCAN_BMEAN, YCAN_BSD};
		float[] greenCan = {GCAN_RMEAN, GCAN_RSD, GCAN_GMEAN, GCAN_GSD, GCAN_BMEAN, GCAN_BSD};
		
		switch(canColor) {
		case 0:
			return redCan;
		case 1:
			return blueCan;
		case 2:
			return yellowCan;
		case 3:
			return greenCan;
			default:
				return new float[0];
		}
	}
	
}
