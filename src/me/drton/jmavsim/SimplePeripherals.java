package me.drton.jmavsim;

public class SimplePeripherals implements Peripherials {

	Peripherial buzzer = new SimpleBuzzer();
	
	@Override
	public Peripherial getBuzzer() {
		return buzzer;
	}

	@Override
	public Peripherial getStatusLed() {
		return null;
	}

}
