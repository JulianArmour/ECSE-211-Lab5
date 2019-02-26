package ca.mcgill.ecse211.lab5.sensors.lightSensor;

import lejos.robotics.SampleProvider;

public class ColourLightSensor {
	
	private SampleProvider LSSampleProvider;
	private float[] LSSample;
	
	public ColourLightSensor(SampleProvider LSSampleProvider, float[] LSSample) {
		this.LSSampleProvider = LSSampleProvider;
		this.LSSample = LSSample;
	}

	public float[] fetchColorSamples(){
	 LSSampleProvider.fetchSample(LSSample, 0);
	 return LSSample.clone();
	}
}
