package frc.robot.commands;

import frc.robot.subsystems.LimelightFrontRight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DetectBall extends Command {
    
    private final LimelightFrontRight limelight;

    public DetectBall(LimelightFrontRight limelight) {
        this.limelight = limelight;

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
    }

    @Override
    public void execute() {
        boolean seeBall = limelight.hasTarget();

        SmartDashboard.putBoolean("Ball Detected", seeBall);

        if (seeBall) {
            SmartDashboard.putNumber("Ball tx", limelight.getTx());
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Ball Detected", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
