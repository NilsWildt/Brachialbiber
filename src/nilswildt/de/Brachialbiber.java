package nilswildt.de;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

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
	protected LineFollower lineFollower;
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
	public Brachialbiber(Port colSensPort, Port gyroSensPort, Port NXTSensPort,
			Port leftMotorPort, Port rightMotorPort) {
		colSensor = new EV3ColorSensor(colSensPort);
		gyro = new EV3GyroSensor(gyroSensPort);
		eiMotor = new NXTMotor(NXTSensPort);
		gyCo = new GyroController(gyro, eiMotor);
		leftMotor = new EV3LargeRegulatedMotor(leftMotorPort);
		rightMotor = new EV3LargeRegulatedMotor(rightMotorPort);
		lineFollower = new LineFollower(colSensor, leftMotor, rightMotor);
	}

	/**
	 * Druckt eine Text passend auf das Display
	 * 
	 * @param text
	 *            Text, der ausgedruckt werden soll
	 * @param Ypos
	 *            Zeile in der der Text beginnt
	 */

	public static void printer(String text, int Ypos) {
		String[] output = text.split(" ");
		int n = output.length;
		int currSplit = 0;
		int Xpos = 0;

		for (int i = Ypos; i < SCREENHIGHT && currSplit != n; i++) {
			LCD.drawString(output[currSplit], Xpos, i);
			Xpos += output[currSplit].length() + 2;
			currSplit++;
			while (currSplit != n
					&& Xpos + output[currSplit].length() <= SCREENWIDTH) {
				LCD.drawString(output[currSplit], Xpos, i);
				Xpos += output[currSplit].length() + 2;
				currSplit++;
			}
			System.out.println();
			Xpos = 0;
		}
	}

	public static void printer(String text) {
		String[] output = text.split(" ");
		int n = output.length;
		int currSplit = 0;
		int Xpos = 0;

		for (int i = 0; i < SCREENHIGHT && currSplit != n; i++) {
			LCD.drawString(output[currSplit], Xpos, i);
			Xpos += output[currSplit].length() + 2;
			currSplit++;
			while (currSplit != n
					&& Xpos + output[currSplit].length() <= SCREENWIDTH) {
				LCD.drawString(output[currSplit], Xpos, i);
				Xpos += output[currSplit].length() + 2;
				currSplit++;
			}
			System.out.println();
			Xpos = 0;
		}
	}

	/*
	 * Writes any Data as String to a File named AAAoutput.txt
	 */
	public static <T> void writeToFile(T data) {
		String stringData = new String(data.toString());
		try {

			File file = new File("AAAoutput.txt");
			// Wenn es das ned gibt, oder macht der das automatisch? TODO!
			if (!file.exists()) {
				file.createNewFile();
			}
			PrintWriter out = new PrintWriter(new BufferedWriter(
					new FileWriter(file)), true);
			out.append(stringData); // oder out.write?
			out.flush();
			out.close();

		} catch (IOException e) {
			System.out.println("I/O Error");
			// System.exit(0); //Roboter soll nicht stehen bleiben...
		}
	}

}
