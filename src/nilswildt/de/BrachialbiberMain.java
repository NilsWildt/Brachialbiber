package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

public class BrachialbiberMain {
	public static void main(String[] args) {
		boolean isBlue = false;
		boolean onlyOnce = false;
		double i = 8.0;
		
		Brachialbiber biber = new Brachialbiber(SensorPort.S1, SensorPort.S2,
				MotorPort.B, MotorPort.A,MotorPort.D);
		Sound.beep();
		Brachialbiber.printer("Press enter to continue");
		Button.waitForAnyPress();
		Delay.msDelay(1000);
		biber.gyCo.initalizeGyroController();
		Delay.msDelay(200);
		biber.lineFollower.initFollower();
		biber.lineFollower.initMotor();
		LCD.clear();
		Sound.beep();
		Delay.msDelay(200);
		LCD.clear();
		//LCD.drawString("Red:" , 0, 0);
		//LCD.drawString("Green:" , 0, 1);
		//LCD.drawString("Blue:" , 0, 2);
		while (Button.ESCAPE.isUp()){// && !isBlue){
			if(Button.UP.isDown()){
				i += 0.5;
				biber.gyCo.setKP(i);
			}
			if(Button.DOWN.isDown()){
				i -= 0.5;
				biber.gyCo.setKP(i);
			}
			/*LCD.clear();
			float[] s = biber.lineFollower.fetchAverageSample();
			LCD.drawString("                 ", 5, 0);
			LCD.drawString("                 ", 7, 1);
			LCD.drawString("                 ", 6, 2);
			LCD.drawInt((int) (s[0]*100.0f), 5, 0);
			LCD.drawInt((int) (s[1]*100.0f), 7, 1);
			LCD.drawInt((int) (s[2]*100.0f), 6, 2);
			Delay.msDelay(100);*/
			biber.lineFollower.drive();
			isBlue = biber.lineFollower.isBlue();
			if(isBlue) onlyOnce = true;
			if(onlyOnce) biber.gyCo.setMotion();
		}	
	}
}
