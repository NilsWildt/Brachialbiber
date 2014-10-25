package de.nilswildt.de;

import lejos.hardware.port.SensorPort;

public class BrachialbiberMain {
	public static void main(String[] args) {
		Brachialbiber biber = new Brachialbiber(SensorPort.S1);
		biber.initialize();
		biber.view();
	}
}