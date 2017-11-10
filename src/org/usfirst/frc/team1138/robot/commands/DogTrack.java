package org.usfirst.frc.team1138.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1138.robot.Robot;

public class DogTrack extends Command{
    private TurnWithGyro turn = null;
    public DogTrack() {
        // Use requires() here to declare subsystem dependencies
        //requires(Robot.SUB_DRIVE_BASE);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        double targetAngle = SmartDashboard.getNumber("setAngle", 0);
        turn = new TurnWithGyro();
        turn.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
//        double targetAngle = SmartDashboard.getNumber("setAngle", 0);
//        if (targetAngle != 0 && turn == null) {
//            turn = new TurnWithGyro(targetAngle);
//            turn.start();
//        }
//
//        if(targetAngle != 0 && turn != null && turn.isFinished()) {
//            turn.cancel();
//            turn = new TurnWithGyro(targetAngle);
//            turn.start();
//        }

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        if (turn != null && !turn.isFinished()){
            turn.end();
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
