package ca.mcgill.ecse211.localization;

import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to say something on the screen
 */
public class Display {
  private TextLCD lcd;

  /**
   * This is the class constructor
   */
  public Display(TextLCD lcd) {
    this.lcd = lcd;
  }
  
  public void run() {    
    lcd.clear();
    lcd.drawString("Don't look at me", 0, 0);
    lcd.drawString(" >_< ", 0, 1);
  }
}
