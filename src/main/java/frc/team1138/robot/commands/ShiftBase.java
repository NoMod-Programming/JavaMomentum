package frc.team1138.robot.commands;

import frc.team1138.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Zheyuan Hu
 * @version 1.0.0
 * This Command requires Robot.SUB_DRIVE_BASE
 */
public class ShiftBase extends Command{
    public ShiftBase() {
		// TODO Auto-generated constructor stub
		requires(Robot.SUB_DRIVE_BASE);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.SUB_DRIVE_BASE.toggleShift();
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
