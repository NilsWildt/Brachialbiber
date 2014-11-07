package nilswildt.de;

import java.io.File;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

public class BrachialbiberMain {
	public static void main(String[] args) {

		Brachialbiber biber = new Brachialbiber(SensorPort.S1, SensorPort.S2,
				MotorPort.B, MotorPort.A, MotorPort.D);

		

		Brachialbiber.printer("Press enter to continue");
		Button.waitForAnyPress();
		Delay.msDelay(1000);
		biber.gyCo.initalizeGyroController();
		Delay.msDelay(200);
		biber.lineFollower.initFollower();
		biber.lineFollower.initMotor();

		LCD.clear();

		// -------------------------------------------------------------
		while (Button.ESCAPE.isUp()) {
			biber.lineFollower.drive();
			biber.gyCo.setMotion();
		}
		biber.gyCo.interruptMotion();
		biber.lineFollower.stopMotor();
		File musicFile = new File("quali2.wav");
		Sound.setVolume(100);
		Sound.playSample(musicFile, 100);
		Button.waitForAnyPress();
		System.exit(0);
	}
}
