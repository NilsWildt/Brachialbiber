package nilswildt.de;

import javax.sound.sampled.Line;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;

public class Brachialbiber {
	protected EV3ColorSensor colSensor;
	private NXTMotor eiMotor;
	private EV3GyroSensor gyro;
	protected GyroController gyCo;
	protected LineController lineCo;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;


	private static int SCREENWIDTH = 17;
	private static int SCREENHIGHT = 7;

	/**
	 * Contructor
	 * 
	 * @param colSenPort
	 *            Port of the color sensor
	 * @param b
	 * @param gyroSenPort
	 */
	public Brachialbiber(Port colSensPort, Port gyroSensPort, Port NXTSensPort, Port leftMotorPort, Port rightMotorPort) {
		colSensor = new EV3ColorSensor(colSensPort);
		gyro = new EV3GyroSensor(gyroSensPort);
		eiMotor = new NXTMotor(NXTSensPort);
		gyCo = new GyroController(gyro, eiMotor);
		leftMotor = new EV3LargeRegulatedMotor(leftMotorPort);
		rightMotor = new EV3LargeRegulatedMotor(rightMotorPort);
		
		lineCo = new LineController(colSensor, leftMotor, rightMotor);
	}

	/**
	 * Druckt eine Text passend auf das Display
	 * 
	 * @param text
	 *            Text, der ausgedruckt werden soll
	 */

	public static void printer(String text) {
		String[] output = text.split(" ");
		int n = output.length;
		int currSplit = 0;
		int Xpos = 0;
		int leftSpace = SCREENWIDTH;
		for (int i = 0; i < SCREENHIGHT && currSplit != n; i++) {
			LCD.drawString(output[currSplit], Xpos, i);
			Xpos += output[currSplit].length() + 1;
			currSplit++;
			while (currSplit != n
					&& Xpos + output[currSplit].length() <= SCREENWIDTH) {
				LCD.drawString(output[currSplit], Xpos, i);
				Xpos += output[currSplit].length() + 1;
				currSplit++;
			}
			System.out.println();
			Xpos = 0;
		}
	}

}
