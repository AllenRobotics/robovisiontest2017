package org.usfirst.frc.team5417.robot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PID implements PIDSource, PIDOutput {
	private PIDController pidController;
	private double currentSample = 0;
	private double currentOutput = 0;
	private PIDSourceType sourceType;
	
	// a good start for P, I, and D would be (1, 20000, 0)
	public PID(double Kp, double Ki, double Kd, PIDSourceType sourceType){
		this.sourceType = sourceType;
		this.pidController = new PIDController(Kp, Ki, Kd, this, this);
	}

	public PID(double Kp, double Ki, double Kd, PIDSourceType sourceType, double setpoint){
		this.sourceType = sourceType;
		this.pidController = new PIDController(Kp, Ki, Kd, this, this);
		this.pidController.setSetpoint(setpoint);
	}

	public void newSample(double sample){
		currentSample = sample;
	}
	
	public double getOutput(){
		return currentOutput;
	}
	
	public void setSetpoint(double setpoint) {
		pidController.setSetpoint(setpoint);
	}
	
	
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		currentOutput = output;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		sourceType = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return sourceType;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return currentSample;
	}

}
