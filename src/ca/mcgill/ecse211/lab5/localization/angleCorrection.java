package ca.mcgill.ecse211.lab5.localization;

import ca.mcgill.ecse211.lab5.navigator.MovementController;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.DifferentialLightSensor;

public class angleCorrection {

	private DifferentialLightSensor dLTright;
	private DifferentialLightSensor dLTleft;
	private MovementController movCon;
	private Odometer odo;
	private static int POLLING_PERIOD = 20;
	private static int DIFFERENTIAL_THRESHOLD = 6;
	

	//constructor
	public angleCorrection(DifferentialLightSensor diffLTright, DifferentialLightSensor diffLTleft,
			MovementController movCon,Odometer odo) {
		this.dLTleft=diffLTleft;
		this.dLTright=diffLTright;
		this.movCon=movCon;
		this.odo=odo;
	}

	public void quickThetaCorrection() {
	    boolean RLineDetected=false;
	    boolean LLineDetected=false;
		
		movCon.driveForward(80);
		while (!RLineDetected || !LLineDetected) {
			//poll right sensor
			int deltaR = dLTright.getDeltaL();
			//poll left sensor
			int deltaL = dLTleft.getDeltaL();

			if (Math.abs(deltaR) > DIFFERENTIAL_THRESHOLD) {
				RLineDetected = true;
				movCon.stopMotor(true);
			}

			if (Math.abs(deltaL) > DIFFERENTIAL_THRESHOLD) {
				LLineDetected = true;
				movCon.stopMotor(false);
			}
			
			try {
                Thread.sleep(POLLING_PERIOD);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
		
		}
		odo.setTheta(movCon.roundAngle());
	}
}
