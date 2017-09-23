package org.usfirst.frc.team1138.robot.commands;

import org.usfirst.frc.team1138.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnWithGyro extends PIDCommand{
	private static double P = 1.0 ,I =0.0 ,D = 0.0; 
	private PIDController turnController; 

	public TurnWithGyro(double angle) {
		super("Turn Angle", P, I, D);
		requires(Robot.SUB_DRIVE_BASE);
		turnController = this.getPIDController(); 
		turnController.setInputRange(-180, 180);
		turnController.setOutputRange(-1, 1);
		turnController.setAbsoluteTolerance(1.0);
		turnController.setContinuous(true);
		turnController.setSetpoint(angle);
		LiveWindow.addActuator("Please Work", "PID CONTROLLER", turnController);
		System.out.println("Inited");
		Robot.SUB_DRIVE_BASE.resetGyro();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		//initialAngle = Robot.SUB_DRIVE_BASE.getAngle(); 
	}
	
	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		//SmartDashboard.putNumber("Error", (Robot.SUB_DRIVE_BASE.getAngle()-initialAngle));
		return (Robot.SUB_DRIVE_BASE.getAngle());
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		if (this.returnPIDInput()-this.getSetpoint() > 0) { // need to turn left
			System.out.println("turn left");
			System.out.println("Left Motor: " + (-output) + "Right Motor: " + (output));
			Robot.SUB_DRIVE_BASE.tankDrive(-output,output);
		}
		else if (this.returnPIDInput()-this.getSetpoint() < 0) { // need to turn right
			System.out.println("turn right");
			System.out.println("Left Motor: " + (output) + "Right Motor: " + (-output));
			Robot.SUB_DRIVE_BASE.tankDrive(output,-output);
		}
		System.out.println("Current Angle: " + Robot.SUB_DRIVE_BASE.getAngle());
		System.out.println("Error: " + (Robot.SUB_DRIVE_BASE.getAngle()-this.getSetpoint()));
		System.out.println("Input: " + this.returnPIDInput());
		System.out.println("On Target: " + turnController.onTarget());
	}

	// Called repeatedly when this Command is scheduled to run

	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		System.out.println("On Target: " + turnController.onTarget());
		return turnController.onTarget();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.SUB_DRIVE_BASE.tankDrive(0,0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
