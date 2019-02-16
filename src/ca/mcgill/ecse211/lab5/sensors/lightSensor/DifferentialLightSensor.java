package ca.mcgill.ecse211.lab5.sensors.lightSensor;

import lejos.robotics.SampleProvider;

public class DifferentialLightSensor extends Thread {
    private int pastData;
    private int deltaL;
    private SampleProvider colorProvider;
    private float[] sampleLSData;

    public DifferentialLightSensor(SampleProvider LSprovider, float[] sampleLS) {
        this.pastData = 100;
        this.colorProvider = LSprovider;
        this.sampleLSData = sampleLS;
        this.deltaL = 0;
    }

    public int getDeltaL() {
        colorProvider.fetchSample(sampleLSData, 0);

        // calculate the difference between current and past light intensity
        deltaL = (int) (100 * sampleLSData[0] - pastData);

        // store the last data in past Data
        pastData = (int) (100 * sampleLSData[0]);

        return deltaL;
    }

}
