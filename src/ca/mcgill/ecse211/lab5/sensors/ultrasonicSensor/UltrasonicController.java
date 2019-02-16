package ca.mcgill.ecse211.lab5.sensors.ultrasonicSensor;

public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();
}