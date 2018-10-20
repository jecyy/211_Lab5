
package ca.mcgill.ecse211.localization;

import lejos.robotics.SampleProvider;

/**
 * This class runs the ultrasonic sensor using a thread
 * and provides the readings
 * @author jecyy
 *
 */
public class UltrasonicPoller extends Thread {
	private SampleProvider us;
	private float[] usData;
	private static int distance;

	public UltrasonicPoller(SampleProvider us, float[] usData) {
		this.us = us;
		this.usData = usData;
	}

	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		while (true) {
			us.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
			if (distance > 200) distance = 200; // if distance is too large, it is set to 200
			if (distance < 5) distance = 5; // if distance is too small, it is set to 5
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
	public static int get_distance() {
		return distance;
	}
}