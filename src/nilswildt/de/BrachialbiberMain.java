package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;

public class BrachialbiberMain {
	public static void main(String[] args) {
		Brachialbiber biber = new Brachialbiber(SensorPort.S1, SensorPort.S2,
				MotorPort.B);
		Sound.beep();
		Brachialbiber.printer("Press any key to continue");
		Button.waitForAnyPress();
		biber.gyCo.initalizeGyroController();
		biber.lineCo.initializeLineController();
		while (Button.ESCAPE.isUp()) {
			biber.lineCo.startMotor();
			biber.lineCo.drive();
			biber.gyCo.setMotion();
		}
	}
}