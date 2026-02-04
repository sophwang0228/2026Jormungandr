package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Retro;

public class AlignToHubBasisVector extends Command {
    //private Limelight 
    private Drivetrain drivetrain;
    private PIDController lateralPIDController, depthPIDController, rotationalPIDController;
    private double lateralP, lateralI, lateralD, lateralFF, 
    depthP, depthI, depthD, depthFF, 
    rotationalP, rotationalI, rotationalD, rotationalFF;
    private double rotationalLowerP; // lower P if error is small, since degrees have larger margin of error
    private double lateralErrorThreshold, depthErrorThreshold, rotationalErrorThreshold, // determines when error is small enough
    rotationalLowerPThreshold; // determines which rotationalP to use
    private Limelight frontRLimelight;

    public AlignToHubBasisVector() {
        frontRLimelight = LimelightFrontRight.getInstance();
        rotationalP = 
        rotationalPIDController = new PIDController(rotationalP, rotationalI, rotationalD);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
