package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.search.Search;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to say something on the screen
 */
public class Display implements Runnable{
	private TextLCD lcd;

	/**
	 * This is the class constructor
	 */
	public Display(TextLCD lcd) {
		this.lcd = lcd;
	}

	public void run() {    
		lcd.clear();
		while (true) {
			lcd.clear();
			if (UltrasonicPoller.get_distance() > 4.5) {
			}
			else {
				lcd.drawString("Object Detected", 0, 0);
				if (Search.findMatch() == 0) lcd.drawString("Blue", 0, 1);
				else if(Search.findMatch() == 1) lcd.drawString("Green", 0, 1);
				else if(Search.findMatch() == 2) lcd.drawString("Yellow", 0, 1);
				else if(Search.findMatch() == 3) lcd.drawString("Orange", 0, 1);
			}

			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
			}
		}
	}
}

