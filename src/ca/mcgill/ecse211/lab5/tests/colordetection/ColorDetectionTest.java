package ca.mcgill.ecse211.lab5.tests.colordetection;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class ColorDetectionTest {

		  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
		  private static final Port LPort = LocalEV3.get().getPort("S1");
		  private static EV3ColorSensor lSensor;
		  private static SampleProvider sampleProviderL;
		  private static float[] samplesL;
		  public static final double leftRadius = 2.0;
		  public static final double rightRadius = 2.0;
		  public static final double track = 8.45;
		  public static final double WHEEL_RAD = 2.0;
		  public static final double TRACK = 8.3;
		  static int buttonChoice;
		  private static float[][] array = new float[100][3];
		  
		  public static void main(String args[]) {
			  
			    (new Thread(){
			    	public void run() {
			    		while (Button.waitForAnyPress() != Button.ID_ESCAPE); // Terminate when escape is pressed
			    	    System.exit(0);
			    	}
			    }).start();
				lSensor = new EV3ColorSensor(LPort);
				sampleProviderL = lSensor.getRGBMode();
				samplesL = new float[sampleProviderL.sampleSize()];
				
				do {
				      // clear the display
				      lcd.clear();

				      // ask the user whether the robot will encounter obstacles or not
				      lcd.drawString("Press button", 0, 0);
				      lcd.drawString("to start", 0, 1);
				      

				      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
				    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
				lcd.clear();
				
				for(int i = 0; i < 100; i++) {
					
					sampleProviderL.fetchSample(samplesL, 0);
					
					if(samplesL[0] <= (0.00098) && samplesL[1] <= (0.00098) && samplesL[2] <= (0.00098)) {
						i--;
					}else {
						array[i] = samplesL;
					}
					
					try {
						Thread.sleep(75);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
				System.out.println("Can: " + ColorDetector.verifyCan(array, 4));
				
		  }
				
		  
}
