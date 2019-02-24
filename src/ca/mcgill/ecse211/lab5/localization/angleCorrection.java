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
	private static float FIRST_DIFFERENCE_THRESHOLD = 4.0f;
	private static float SECOND_DIFFERENCE_THRESHOLD = 1.5f;
	

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
	        
	        float threshold;
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
            float deltaR = 0f;
            float deltaL = 0f;
            while (!RLineDetected || !LLineDetected) {
                //poll right sensor
                if (!RLineDetected) {
                    deltaR = (dLTright.getDeltaL());
                }
                //poll left sensor
                if (!LLineDetected) {
                    deltaL = (dLTleft.getDeltaL());
                }

                if (Math.abs(deltaR) > threshold) {
                    RLineDetected = true;
//                    System.out.println("right sensor detected line");
//                    System.out.println(RLineDetected);
//                    System.out.println(LLineDetected);
                    movCon.stopMotor(true, true);
//                    System.out.println(deltaR);
                }

                if (Math.abs(deltaL) > threshold) {
                    LLineDetected = true;
//                    System.out.println("left sensor detected line");
//                    System.out.println(RLineDetected);
//                    System.out.println(LLineDetected);
                    movCon.stopMotor(false, true);
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
	    
	    movCon.resetMotorSpeeds();
        odo.setTheta(movCon.roundAngle());
//        System.out.println(odo.getXYT()[2]);
	}
}
