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
	private static int DIFFERENCE_THRESHOLD = 2;
	

	//constructor
	public angleCorrection(DifferentialLightSensor diffLTright, DifferentialLightSensor diffLTleft,
			MovementController movCon,Odometer odo) {
		this.dLTleft=diffLTleft;
		this.dLTright=diffLTright;
		this.movCon=movCon;
		this.odo=odo;
	}

	public void quickThetaCorrection() {
	    for (int i = 0; i < 2; i++) {
	        boolean RLineDetected=false;
	        boolean LLineDetected=false;
	        
            // get rid of old light sensor data
            dLTright.flush();
            dLTleft.flush();
            movCon.driveForward(80);
            while (!RLineDetected || !LLineDetected) {
                //poll right sensor
                int deltaR = dLTright.getDeltaL();
                //poll left sensor
                int deltaL = dLTleft.getDeltaL();

                if (Math.abs(deltaR) > DIFFERENCE_THRESHOLD) {
                    RLineDetected = true;
                    movCon.stopMotor(true, true);
                    System.out.println(deltaR);
                }

                if (Math.abs(deltaL) > DIFFERENCE_THRESHOLD) {
                    LLineDetected = true;
                    movCon.stopMotor(false, true);
                    System.out.println(deltaL);
                }

                try {
                    Thread.sleep(POLLING_PERIOD);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
            if (i == 0) {
                movCon.driveDistance(-3);
            }
        }
        odo.setTheta(movCon.roundAngle());
	}
}
