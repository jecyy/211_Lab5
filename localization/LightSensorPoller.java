package ca.mcgill.ecse211.localization;

import lejos.robotics.SampleProvider;

/**
 * This class provides the light sensor readings
 * @author jecyy
 *
 */
public class LightSensorPoller extends Thread {
	private static float light = 0;
	private SampleProvider myLightSample;
	private float[] sampleLight;
	
	public LightSensorPoller(SampleProvider myLightSample, float[] sampleLight) {
		this.myLightSample = myLightSample;
		this.sampleLight = sampleLight;
	}
	
	public void run() {
		while (true) {
			myLightSample.fetchSample(sampleLight, 0); // activate the light sensor
			light = sampleLight[0] * 1000; // get the light sensor reading
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			}
		}
	}

	/**
	 * This method is to read the current sensor reading
	 * @return
	 */
	public static float get_light() {
		return light;
	}
}
