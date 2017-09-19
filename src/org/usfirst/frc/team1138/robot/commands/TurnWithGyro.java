package org.usfirst.frc.team1138.robot.commands;

import org.usfirst.frc.team1138.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnWithGyro extends PIDCommand{
	private static double P = 1.0 ,I =0.0 ,D = 0.0; 
	private PIDController turnController; 
	private double initialAngle = 0.0; 
	public TurnWithGyro(double angle) {
		super("Turn Angle", P, I, D); 
		requires(Robot.SUB_DRIVE_BASE);
		turnController = getPIDController(); 
		turnController.setInputRange(-180, 180);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(1.0);
		turnController.setContinuous(true);
		turnController.setSetpoint(angle);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		initialAngle = Robot.SUB_DRIVE_BASE.getAngle(); 
	}
	
	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		SmartDashboard.putNumber("Error", (Robot.SUB_DRIVE_BASE.getAngle()-initialAngle));
		return (Robot.SUB_DRIVE_BASE.getAngle()-initialAngle);
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		SmartDashboard.putNumber("Output", Robot.SUB_DRIVE_BASE.getOutput());
		Robot.SUB_DRIVE_BASE.drivePID(output);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return turnController.onTarget();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
	
	public PIDController getPID() {
		return getPIDController();
	}
}
