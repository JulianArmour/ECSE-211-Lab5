package ca.mcgill.ecse211.lab5.localization;

import ca.mcgill.ecse211.lab5.navigator.MovementController;
import ca.mcgill.ecse211.lab5.odometer.Odometer;
import ca.mcgill.ecse211.lab5.sensors.lightSensor.DifferentialLightSensor;
import lejos.hardware.lcd.LCD;

public class angleCorrection {

	private DifferentialLightSensor dLTright;
	private DifferentialLightSensor dLTleft;
	private MovementController movCon;
	private Odometer odo;
	private static int POLLING_PERIOD = 20;
	private static int FIRST_DIFFERENCE_THRESHOLD = 280;
	private static int SECOND_DIFFERENCE_THRESHOLD = 250;
	

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
	        
	        int threshold;
	        if (i == 0) {
	            threshold = FIRST_DIFFERENCE_THRESHOLD;
	        } else {
	            threshold = SECOND_DIFFERENCE_THRESHOLD;
	        }
	        
            // get rid of old light sensor data
            dLTright.flush();
            dLTleft.flush();
            if (i == 0) {
                movCon.driveForward(100);
            }
            else {
                movCon.driveForward(30);
            }
            
            // outside of while for testing purposes, TODO put back in while loop
            int deltaR = 0;
            int deltaL = 0;
            while (!RLineDetected || !LLineDetected) {
                //poll right sensor
                if (!RLineDetected) {
                    deltaR = (int) (dLTright.getDeltaL() * 100);
                }
                //poll left sensor
                if (!LLineDetected) {
                    deltaL = (int) (dLTleft.getDeltaL() * 100);
                }

                if (Math.abs(deltaR) > threshold) {
                    RLineDetected = true;
//                    System.out.println("right sensor detected line");
//                    System.out.println(RLineDetected);
//                    System.out.println(LLineDetected);
                    movCon.stopMotor(true, false);
//                    System.out.println(deltaR);
                }

                if (Math.abs(deltaL) > threshold) {
                    LLineDetected = true;
//                    System.out.println("left sensor detected line");
//                    System.out.println(RLineDetected);
//                    System.out.println(LLineDetected);
                    movCon.stopMotor(false, false);
//                    System.out.println(deltaL);
                }

                try {
                    Thread.sleep(POLLING_PERIOD);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
            System.out.println("left: "+deltaL+" right: "+deltaR);
            if (i < 1) {
                movCon.driveDistance(-3.0);
            }
        }
	    
	    
        odo.setTheta(movCon.roundAngle());
//        System.out.println(odo.getXYT()[2]);
	}
}
