package ca.mcgill.ecse211.lab5.sensors.lightSensor;

import lejos.robotics.SampleProvider;

public class DifferentialLightSensor extends Thread {
    private float pastSample;
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
    public float getDeltaL() {
        
        colorProvider.fetchSample(sampleLSData, 0);

        // calculate the difference between current and past light intensity
        float deltaL = (100 * sampleLSData[0] - pastSample);//TODO make it 1000 * ...

        // store the last data in past Data
        pastSample = 100 * sampleLSData[0];

        return deltaL;
    }

}
