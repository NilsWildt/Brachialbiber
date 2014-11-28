package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

public class BrachialbiberMain {
	public static void main(String[] args) {
		Brachialbiber biber = new Brachialbiber(SensorPort.S1, SensorPort.S2, SensorPort.S3,
				MotorPort.B, MotorPort.A, MotorPort.D);
		
		LineFollower follower = biber.lineFollower;
		GyroController controller = biber.gyCo;
		
		//Initialisierungen
		Delay.msDelay(1000);
		controller.initalizeGyroController();
		Delay.msDelay(200);
		follower.initFollower();
		follower.initMotor();
		LCD.clear();
		// -------------------------------------------------------------

		while (Button.ESCAPE.isUp()) {
			follower.drive();
			controller.setMotion();
		}
		
		/*
		// Play Music and stop!
		biber.gyCo.interruptMotion();
		biber.lineFollower.stopMotor();
		File musicFile = new File("quali2.wav");
		Sound.setVolume(100);
		Sound.playSample(musicFile, 100);
		Button.waitForAnyPress();*/
		System.exit(0);
	}
}
