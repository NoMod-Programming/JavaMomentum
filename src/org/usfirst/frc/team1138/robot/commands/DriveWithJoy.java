package org.usfirst.frc.team1138.robot.commands;

import org.usfirst.frc.team1138.robot.OI;
import org.usfirst.frc.team1138.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class DriveWithJoy extends Command{
	private OI oi; 
	public DriveWithJoy() {
		// TODO Auto-generated constructor stub
		requires(Robot.SUB_DRIVE_BASE);
		oi = new OI(); 
	}
	
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// two hand or one hand 
		if (Robot.SUB_DRIVE_BASE.getLiftState() == DoubleSolenoid.Value.kForward)
			Robot.SUB_DRIVE_BASE.tankDrive(oi.getRightController(), oi.getRightController());
		else Robot.SUB_DRIVE_BASE.tankDrive(oi.getLeftController(), oi.getRightController());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true;
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
}
