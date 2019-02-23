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
	private static int DIFFERENCE_THRESHOLD = 290;
	

	//constructor
	public angleCorrection(DifferentialLightSensor diffLTright, DifferentialLightSensor diffLTleft,
			MovementController movCon,Odometer odo) {
		this.dLTleft=diffLTleft;
		this.dLTright=diffLTright;
		this.movCon=movCon;
		this.odo=odo;
	}

	public void quickThetaCorrection() {
	    for (int i = 0; i < 1; i++) {
	        boolean RLineDetected=false;
	        boolean LLineDetected=false;
	        
            // get rid of old light sensor data
            dLTright.flush();
            dLTleft.flush();
            if (i == 0) {
                movCon.driveForward(80);
            }
            else {
                movCon.driveForward(60);
            }
            
            while (!RLineDetected || !LLineDetected) {
                //poll right sensor
                int deltaR = (int) (dLTright.getDeltaL() * 100);
                //poll left sensor
                int deltaL = (int) (dLTleft.getDeltaL() * 100);

                if (Math.abs(deltaR) > DIFFERENCE_THRESHOLD) {
                    RLineDetected = true;
//                    System.out.println("right sensor detected line");
                
//                    System.out.println(RLineDetected);
//                    System.out.println(LLineDetected);
                    
                    movCon.stopMotor(true, false);
                    System.out.println(deltaR);
                }

                if (Math.abs(deltaL) > DIFFERENCE_THRESHOLD) {
                    LLineDetected = true;
//                    System.out.println("left sensor detected line");
//                    System.out.println(RLineDetected);
//                    System.out.println(LLineDetected);
                    movCon.stopMotor(false, false);
                    System.out.println(deltaL);
                }

                try {
                    Thread.sleep(POLLING_PERIOD);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
//            if (i < 2) {
//                movCon.driveDistance(-3.0);
//            }
        }
	    
	    
        odo.setTheta(movCon.roundAngle());
//        System.out.println(odo.getXYT()[2]);
	}
}
