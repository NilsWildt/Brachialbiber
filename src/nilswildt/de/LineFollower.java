package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;

public class LineFollower {
	private EV3ColorSensor sensor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private final static int acceleration = 1000;
	private float[] sample;
	private SampleProvider rgbMode;
	private SampleProvider average;
	private float speed;
	private double brightValue; // Höchster Helligkeitswert
	private double darkValue; // Höchster Schwarzwert (Linie)
	private double midValue; // Mitte zwischen Hell und dunkel
	private double currentValue; // Immer der aktuelle Helligkeitswert
	private double error;
	private double turn;

	private final double KC = 5.0;
	private final double PC = 0.66275; // Periodendauer
	private final double dT = 0.0204; // Durchlauf

	private double KP;
	private double KI;
	private double KD;

	private double speedLeft;
	private double speedRight;

	private double integral;
	private double derivative;
	private double lastError;
	
	private double range;
	
	private double KK = 1.1;

	public LineFollower(EV3ColorSensor sensor,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.sensor = sensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		rgbMode = this.sensor.getRGBMode();
		speed = 278.0f;// Math.max(leftMotor.getMaxSpeed(), rightMotor.getMaxSpeed())/3.0f; // Acceleration
		KP = 4.0;// 0.6*KC = 3.0; //5.2
		KI = 0.05;// 0.396;//2.0*KP*dT/PC = 0.184685;
		KD = 12.0;// KP*PC/(8.0*dT) = 12.1829;
		sample = new float[4]; // rgb,intensity at [3]
		integral = 0.0;
		derivative = 0.0;
		lastError = 0.0;
	}

	public void initFollower() {
		double value;
		LCD.clear();

		LCD.drawString("Calibrate Brigth-Value: (Press ENTER-Button)", 0, 0);
		while (Button.ENTER.isUp()) {
			brightValue = (double) fetchAverageSample()[3];
		}
		Delay.msDelay(500);
		LCD.clear();
		LCD.drawString("Calibrate Dark-Value: (Press ENTER-Button)", 0, 0);
		while (Button.ENTER.isUp()) {
			darkValue = (double) fetchAverageSample()[3];
		}
		Delay.msDelay(500);

		midValue = (brightValue + darkValue) / 2.0;
		LCD.clear();
		Brachialbiber.printer("Justiere auf 0");
		Delay.msDelay(1000);
		LCD.clear();
		while (Button.ENTER.isUp()) {
			value = Math.round((fetchAverageSample()[3] - midValue) * 100.0) / 100.0;
			System.out.println(value);
		}

		range = Math.abs(brightValue - darkValue) / 2.0;

		// Werte an Strecke anpassen
		// KP = 10*(25)/(brightValue - darkValue);
		// Uncomment to disable I and D Term!
		// KI = 2.0*KP*dT/PC;
		// KD = KP*PC/(8.0*dT);
		LCD.clear();
	}

	public void initMotor() {
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);

		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		leftMotor.forward();
		rightMotor.forward();
	}

	public void drive() {
		currentValue = fetchAverageSample()[3];

		// P-Part
		error = midValue - currentValue;
		error = Math.signum(error)
				* 1.33
				* range
				/ (1 + Math.pow(Math.E, -0.03 * 1.08 * range
						* (Math.abs(error) - 1.35 * range / 2.0)));

		// I-Part
		// if(Math.signum(error) != Math.signum(integral)) integral = 0.0;
		integral = (2.0 / 3.0) * integral + error; // (2.0/3.0)*

		// D-Part
		derivative = error - lastError;

		turn = KP * error + KI * integral + KD * derivative;
		speedLeft = Math.round(Math.max(Math.min(speed - turn, 2 * speed), 1));
		speedRight = Math.round(Math.max(Math.min(speed + turn, 2 * speed), 1));

		leftMotor.setSpeed((int) speedLeft);
		rightMotor.setSpeed((int) speedRight);

		lastError = error;
		LCD.clear();

	}

	public float[] fetchSample() {
		rgbMode.fetchSample(sample, 0);
		for (int i = 0; i < sample.length - 1; i++) {
			sample[i] = sample[i] * 100f;
		}

		sample[3] = (float) Math.sqrt((Math.pow(sample[0], 2)
				+ Math.pow(sample[1], 2) + Math.pow(2 * sample[2], 2)));
		return sample;
	}

	public float[] fetchAverageSample() {
		average = new MeanFilter(rgbMode, 5);
		average.fetchSample(sample, 0);
		for (int i = 0; i < sample.length - 1; i++) {
			sample[i] = sample[i] * 100f;
		}

		sample[3] = (float) Math.sqrt((Math.pow(sample[0], 2)
				+ Math.pow(sample[1], 2) + Math.pow(2 * sample[2], 2))); // TODO ÃœBERARBEITEN
		return sample;
	}

	public boolean isBlue() {
		sample = fetchAverageSample();
		if (sample[0] < sample[2] * 1.25f)
			return true;
		return false;
	}

	public double getKP() {
		return KP;
	}

	public void setKP(double kp) {
		this.KP = kp;
	}

	public double getKI() {
		return KI;
	}

	public void setKI(double ki) {
		this.KI = ki;
	}

	public double getKD() {
		return KD;
	}

	public void setKD(double kd) {
		this.KD = kd;
	}

	public double getCurrentError() {
		return error;
	}

	public void stopMotor() {
		leftMotor.flt();
		rightMotor.flt();
	}

}
