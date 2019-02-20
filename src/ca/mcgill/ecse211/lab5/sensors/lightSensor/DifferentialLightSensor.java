package ca.mcgill.ecse211.lab5.sensors.lightSensor;

import lejos.robotics.SampleProvider;

public class DifferentialLightSensor extends Thread {
    private int pastSample;
    private SampleProvider colorProvider;
    private float[] sampleLSData;

    public DifferentialLightSensor(SampleProvider LSprovider, float[] sampleLS) {
        this.pastSample = 0;
        this.colorProvider = LSprovider;
        this.sampleLSData = sampleLS;
        getDeltaL(); // get an initial intensity
    }
    
    /**
     * Fetches new samples to get rid of old ones.
     */
    public void flush() {
        getDeltaL();
    }

    /**
     * 
     * @return the difference between two sequential light sensor sample polls.
     */
    public int getDeltaL() {
        
        colorProvider.fetchSample(sampleLSData, 0);

        // calculate the difference between current and past light intensity
        int deltaL = (int) (100 * sampleLSData[0] - pastSample);

        // store the last data in past Data
        pastSample = (int) (100 * sampleLSData[0]);

        return deltaL;
    }

}
