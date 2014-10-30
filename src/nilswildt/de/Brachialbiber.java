package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Brachialbiber {
	private EV3ColorSensor colSensor;
	private NXTMotor eiMotor;
	private EV3GyroSensor gyro;
	protected GyroController gyCo;
	private float[] colorSensorSample;
	private SampleProvider redMode;
	// private SampleProvider rgbMode;
	private float brightValue;
	private float darkValue;

	private int SCREENWIDTH = 17;
	private int SCREENHIGHT = 7;
	
	
	/**
	 * Contructor
	 * 
	 * @param colSenPort
	 *            Port of the color sensor
	 * @param b 
	 * @param gyroSenPort 
	 */
	public Brachialbiber(Port colSensPort, Port gyroSensPort, Port NXTSensPort) {
		colSensor = new EV3ColorSensor(colSensPort);
		gyro = new EV3GyroSensor(gyroSensPort);
		eiMotor = new NXTMotor(NXTSensPort);
		gyCo = new GyroController(gyro, eiMotor);
		redMode = colSensor.getRedMode();
		// rgbMode = colSensor.getRGBMode();
		colorSensorSample = new float[redMode.sampleSize()];
	}
	
	/**
	 * Druckt eine Text passend auf das Display 
	 * 
	 * @param text  Text, der ausgedruckt werden soll
	 */
	public void printer(String text){
		String[] output = text.split(" ");
		int n = output.length;
		int currSplit = 0;
		int Xpos = 0;
		int leftSpace = SCREENWIDTH;
		for(int i=0; i<SCREENHIGHT && currSplit != n; i++){
			LCD.drawString(output[currSplit], Xpos, i);
			Xpos += output[currSplit].length() + 1;
			currSplit++;
			while(currSplit != n && Xpos + output[currSplit].length() <= SCREENWIDTH){
				LCD.drawString(output[currSplit], Xpos, i);
				Xpos += output[currSplit].length() + 1;
				currSplit++; 
			}
			System.out.println();
			Xpos = 0;
		}
	}





	/**
	 * Calibrate the ColorSensor
	 */
	public void initialize() {
		/* Read the High Value */
		LCD.drawString("Calibrate bright", 0, 0);
		Button.waitForAnyPress();
		this.brightValue = fetchCurrentValue(); // TODO Mitteln!
		LCD.clear();
		/* Read the Low Value */
		LCD.drawString("Calibrate dark", 0, 0);
		Button.waitForAnyPress();
		this.darkValue = fetchCurrentValue();
		LCD.clear();
		System.out.println("High: " + Float.toString(brightValue) + '\n'
				+ "Low: " + Float.toString(darkValue));
		Delay.msDelay(2000);
	}

	protected void view() {
		LCD.drawString("Zeige RedMode Samples", 0, 0);
		Button.waitForAnyPress();
		LCD.clear();
		// First: redmode:
		while (!Button.ESCAPE.isDown()) {
			System.out.println(fetchCurrentValue());
			Delay.msDelay(10);
		}

	}

	/**
	 * Fetches the current reflected light value.
	 * 
	 * @return reflected light value between 0 and 100
	 */
	private float fetchCurrentValue() {
		redMode.fetchSample(colorSensorSample, 0);
		return colorSensorSample[0] * 100; // In percent!
	}

	public void close() {
		LCD.clear();
		System.out.println("Finished!");
		Delay.msDelay(1000);
		Sound.beep();
		colSensor.close();
		System.exit(1);
	}

}
