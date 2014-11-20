package nilswildt.de;

import java.io.File;
import java.util.Date;

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

		/*
		 * Blue stuff:
		 */
		int blueCount = 0;
		boolean blueFlag = false;
		Sound.setVolume(5); // Damit es nicht so laut Piept.
		long startTime = System.currentTimeMillis();
		long elapsedTime = 0L;

		while (Button.ESCAPE.isUp()) {
			// TODO Werte von Klassen abgreifen, um nicht jedesmal hier hardcoden zu müssen.
			if (biber.lineFollower.isBlue()) {
				++blueCount;
				elapsedTime = (new Date()).getTime() - startTime;
				if (blueCount >= 10 && !blueFlag && elapsedTime >= (3 * 1000)) {
					System.out.println(elapsedTime);
					Sound.beep();
					blueFlag = true;
					blueCount = 0;
					biber.gyCo.setGyroKP(4.0); // davor auf 0...
					biber.lineFollower.setKP(1.0);
					startTime=System.currentTimeMillis();
				} else if (blueCount >= 10 && blueFlag
						&& elapsedTime >= (3 * 1000)) {
					Sound.beep();
					blueCount = 0;
					blueFlag = false;
					startTime=System.currentTimeMillis();
					biber.gyCo.setGyroKP(0); // davor auf 0...
					biber.lineFollower.setKP(5.2);
				}
				if (blueCount == Integer.MAX_VALUE)
					blueCount = 0; // Für den Notfall
			}

			biber.lineFollower.drive();
			biber.gyCo.setMotion();
		}
		System.exit(0);
		// Play Music and stop!
		biber.gyCo.interruptMotion();
		biber.lineFollower.stopMotor();
		File musicFile = new File("quali2.wav");
		Sound.setVolume(100);
		Sound.playSample(musicFile, 100);
		Button.waitForAnyPress();
		System.exit(0);
	}
}
