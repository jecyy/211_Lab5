package ca.mcgill.ecse211.search;

import lejos.robotics.SampleProvider;

/**
 * This class runs the color sensor and provides the readings
 * @author jecyy
 *
 */
public class ColorSensorPoller extends Thread {
	private static float R, G, B;
	private SampleProvider myColorSample;
	private float[] sampleColor;
	
	public ColorSensorPoller(SampleProvider myColorSample, float[] sampleColor) {
		this.myColorSample = myColorSample;
		this.sampleColor = sampleColor;
	}
	
	public void run() {
		while (true) {
			myColorSample.fetchSample(sampleColor, 0); // activate the color sensor
			R = sampleColor[0] * 1000; // get the color sensor reading
			G = sampleColor[1] * 1000;
			B = sampleColor[2] * 1000;
			try {
				Thread.sleep(15);
			} catch (Exception e) {
			}
		}
	}

	/**
	 * This method is to read the current sensor reading
	 * @return
	 */
	public static float getR() {
		return R;
	}
	
	public static float getG() {
		return G;
	}
	
	public static float getB() {
		return B;
	}
}
