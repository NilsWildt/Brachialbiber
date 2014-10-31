package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;

public class BrachialbiberMain {
	public static void main(String[] args) {
		Brachialbiber biber = new Brachialbiber(SensorPort.S1, SensorPort.S2,
				MotorPort.B, MotorPort.A,MotorPort.D);
		Sound.beep();
		Brachialbiber.printer("Press  enter to continue");
		Button.waitForAnyPress();
		biber.gyCo.initalizeGyroController();
		biber.lineCo.initializeLineController();
		biber.lineCo.startMotor();
		LCD.clear();
		Brachialbiber.printer("I'm working...");
		while (Button.ESCAPE.isUp()) {
			biber.lineCo.drive();
			biber.gyCo.setMotion();
		}
	}
}