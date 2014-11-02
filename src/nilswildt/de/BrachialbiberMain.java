package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

public class BrachialbiberMain {
	public static void main(String[] args) {
		long time;
		
		Brachialbiber biber = new Brachialbiber(SensorPort.S1, SensorPort.S2,
				MotorPort.B, MotorPort.A,MotorPort.D);
		Sound.beep();
		Brachialbiber.printer("Press enter to continue");
		Button.waitForAnyPress();
		biber.gyCo.initalizeGyroController();
		biber.lineFollower.initFollower();
		biber.lineFollower.initMotor();
		LCD.clear();
		Sound.beep();
		
		while (Button.ESCAPE.isUp() || biber.lineFollower.isBlue()){
			/*float[] s = biber.lineFollower.fetchAverageSample();
			LCD.clear();
			LCD.drawString("Red:" , 0, 0);
			LCD.drawInt((int) (s[0]*100.0f), 5, 0);
			LCD.drawString("Green:" , 0, 1);
			LCD.drawInt((int) (s[1]*100.0f), 7, 1);
			LCD.drawString("Blue:" , 0, 2);
			LCD.drawInt((int) (s[2]*100.0f), 6, 2);
			Delay.msDelay(200);*/
			biber.lineFollower.drive();
			//biber.gyCo.setMotion();
		}
		
	}
}
