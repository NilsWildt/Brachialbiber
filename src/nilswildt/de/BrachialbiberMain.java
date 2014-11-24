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

		/*
		 * Blue stuff: Beachte: KP WERTE NICHT VERGESSEN Wie viele count?
		 */
		int blueCount = 0;
		boolean blueFlag = false;
		// Sound.setVolume(5); // Damit es nicht so laut Piept.
		long startTime = System.currentTimeMillis();
		long elapsedTime = 0L;

		while (Button.ESCAPE.isUp()) {
			// TODO Werte von Klassen abgreifen, um nicht jedesmal hier hardcoden zu müssen.
			follower.drive();
			//controller.setMotion();
			
			/*if (false) { // biber.lineFollower.isBlue()
				++blueCount;
				elapsedTime = (new Date()).getTime() - startTime;
				if (blueCount >= 1 && !blueFlag && elapsedTime >= (3 * 1000)) {
					System.out.println(elapsedTime);
					Sound.beep();
					blueFlag = true;
					blueCount = 0;
					biber.gyCo.setGyroKP(3.8); // davor auf 0...
					biber.lineFollower.setKP(2.0); // KP auf dem Hügel runter setzen!
					startTime = System.currentTimeMillis();
				} else if (blueCount >= 1  && blueFlag
						&& elapsedTime >= (3 * 1000)) {
					Sound.beep();
					blueCount = 0;
					blueFlag = false;
					startTime = System.currentTimeMillis();
					biber.gyCo.setGyroKP(0); // davor auf 0...
					biber.lineFollower.setKP(5.2);
				}
				*/
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
