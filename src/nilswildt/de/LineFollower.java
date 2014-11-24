package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;

public class LineFollower {

	private EV3ColorSensor sensor;
	private EV3TouchSensor touchSensor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	private final static int acceleration = 1000;
	
	private float[] sample;
	private float[] isTouched;
	private SampleProvider redMode;
	private SampleProvider average;
	private float speed;
	private double brightValue; // Höchster Helligkeitswert
	private double darkValue; // Höchster Schwarzwert (Linie)
	private double midValue; // Mitte zwischen Hell und dunkel
	private double range;
	private double currentValue; // Immer der aktuelle Helligkeitswert
	private double error;
	private double turn;
	
	private final double KC = 5.0;
	private final double PC = 0.66275; //Periodendauer
	private final double dT = 0.0204; //Durchlauf 
	
	private double KP;
	private double KI;
	private double KD;
	private double maxError;
	private double errorChange;
	private double errorShift;
	
	private double speedLeft;
	private double speedRight;
	
	private double integral;
	private double derivative;
	private double lastError;
	
	double KK = 1.1;
	
	public LineFollower(EV3ColorSensor sensor, EV3TouchSensor touchSensor,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		this.sensor = sensor;
		this.touchSensor = touchSensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		redMode = this.sensor.getRedMode();
		speed = 350.0f;// Math.max(leftMotor.getMaxSpeed(), rightMotor.getMaxSpeed())/3.0f; // Acceleration // 
		KP = 3.8;// 0.6*KC = 3.0; //5.2		+		KP = 3.2;// 0.6*KC = 3.0; //5.2
		KI = 0.0;// 0.396;//2.0*KP*dT/PC = 0.184685;		+		KI = 0.00;// 0.396;//2.0*KP*dT/PC = 0.184685;
		KD = 10.0;// KP*PC/(8.0*dT) = 12.1829;		+		KD = 10.0;// KP*PC/(8.0*dT) = 12.1829;
		maxError = 1.0;
		errorChange = 1.0;
		errorShift = 1.0;		
		sample = new float[1]; // rgb,intensity at [3]
		isTouched = new float[1];
		integral = 0.0;
		derivative = 0.0;
		lastError = 0.0;
	}
	
	public void initFollower() {
		double value;
		LCD.clear();
		
		LCD.drawString("Calibrate Brigth-Value: (Press ENTER-Button)",0,0);
		do{
			brightValue = (double) fetchAverageSample();
			touchSensor.fetchSample(isTouched, 0);
		}while(isTouched[0] != 1f);
		Delay.msDelay(500);
		LCD.clear();
		LCD.drawString("Calibrate Dark-Value: (Press ENTER-Button)",0,0);
		do{
			darkValue = (double) fetchAverageSample();
			touchSensor.fetchSample(isTouched, 0);
		}while(isTouched[0] != 1f);
		Delay.msDelay(500);
		
		midValue = (brightValue + darkValue) / 2.0;
		LCD.clear();
		Brachialbiber.printer("Justiere auf 0");
		Delay.msDelay(1000);
		LCD.clear();
		do{
			touchSensor.fetchSample(isTouched, 0);
			value = fetchAverageSample() - midValue;//Math.round((fetchAverageSample() - midValue)*100.0)/100.0;
			System.out.println(value);
		}while(isTouched[0] != 1f);
		
		range = Math.abs(brightValue - darkValue)/2.0;
		
		//Werte an Strecke anpassen
		//KP = 10*(25)/(brightValue - darkValue);
		//Uncomment to disable I and D Term!
		//KI = 2.0*KP*dT/PC;
		//KD = KP*PC/(8.0*dT);
		LCD.clear();
	}
	
	public void initMotor(){
			leftMotor.setAcceleration(acceleration);
			rightMotor.setAcceleration(acceleration);
			
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
			
			leftMotor.forward();
			rightMotor.forward();
	}
	
	public void drive() {
		currentValue = fetchAverageSample();
		
		//P-Part
		error = midValue - currentValue;
		error = Math.signum(error)*maxError*range/(1+Math.pow(Math.E, -0.03*errorChange*range*(Math.abs(error) - errorShift*range/2.0)));
		
		//I-Part
		//if(Math.signum(error) != Math.signum(integral)) integral = 0.0;
		integral = (2.0/3.0)*integral + error; //(2.0/3.0)*
		
		//D-Part
		derivative = error - lastError;
		
		turn = KP * error + KI * integral + KD * derivative;
		speedLeft = Math.round(Math.max(Math.min(speed - turn, 2*speed), 1));
		speedRight = Math.round(Math.max(Math.min(speed + turn, 2*speed), 1));
		
		
		leftMotor.setSpeed((int) speedLeft);
		rightMotor.setSpeed((int) speedRight);
		
		lastError = error;
		LCD.clear();

	}
	
	public float fetchSample(){
		redMode.fetchSample(sample, 0);
		sample[0] *= 100f;
		return sample[0];
	}	
	
	public float fetchAverageSample(){
		average = new MeanFilter(redMode, 5);
		average.fetchSample(sample, 0);
		sample[0] *= 100f;
		return sample[0];
	}
	
	public double getKP(){
		return KP;
	}
	
	public void setKP(double kp){
		this.KP = kp;
	}
	
	public double getKI(){
		return KI;
	}
	
	public void setKI(double ki){
		this.KI = ki;
	}
	
	public double getKD(){
		return KD;
	}
	
	public void setKD(double kd){
		this.KD = kd;
	}
	
	public double getCurrentError(){
		return error;
	}
	
	public void stopMotor(){
		leftMotor.flt();
		rightMotor.flt();
	}
	
}
