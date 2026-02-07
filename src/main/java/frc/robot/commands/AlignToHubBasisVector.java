package frc.robot.commands;

import java.net.ContentHandler;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.utils.Constants;

public class AlignToHubBasisVector extends Command {

    // Create a Drivetrain
    private Drivetrain drivetrain;

    // Create PID Controllers
    private PIDController lateralPIDController, depthPIDController, rotationalPIDController;

    // Create the Lateral, Depth, and Rotational PID FF 
    private double lateralP, lateralI, lateralD, lateralFF;
    private double depthP, dpethI, depthD, dpethFF;
    private double rotationalP, rotationalI, rotationalD, rotationalFF;

    // Create Threshold for all 3
    private double rotationalLowerPThreshold,
                    rotationalErrorThreshold; // determines if the rotational error is small enough to use lower P threshold
    private double rotationalLowerP;
    private double rotationalError;
    private double latreralThreshold, depthThreshold; // Unused

    // Create Limelight
    private Limelight frontLimelight;
    
    private double tagAngle;
    


    public AlignToHubBasisVector() {
        drivetrain = Drivetrain.getInstance();
        frontLimelight = LimelightFrontRight.getInstance();

        rotationalP = Constants.HubAlignConstants.kRotationalP; // Set rotational P
        rotationalI = Constants.HubAlignConstants.kRotationalI; // Set rotational I
        rotationalD = Constants.HubAlignConstants.kRotationalD; // Set rotational D

        // Set all rotational thresholds
        rotationalErrorThreshold = Constants.HubAlignConstants.kRotationalErrorThreshold;
        rotationalLowerP = Constants.HubAlignConstants.kRotationalLowerP;
        rotationalLowerPThreshold = Constants.HubAlignConstants.kRotationalLowerPThreshold;

        rotationalPIDController = new PIDController(rotationalP, rotationalI, rotationalD); // Set PID Controller for rotationals

        addRequirements(drivetrain);

        SmartDashboard.putNumber("Target AprilTag ID", 0);

    }

    @Override
    public void initialize() {
        Pose2d tagPose = Limelight.getAprilTagPose((int)SmartDashboard.getNumber("Target AprilTag ID", 0));
        tagAngle = 0; // use this if needed: tagPose.getRotation().getRadians()

        //Be able to change the values on SmartDashboard
        SmartDashboard.getNumber("Rotational P", rotationalP);
        SmartDashboard.getNumber("Rotational I", rotationalI);
        SmartDashboard.getNumber("Rotational D", rotationalD);
        SmartDashboard.getNumber("Rotational FF", rotationalFF);

        SmartDashboard.getNumber("Rotational Error Threshold", rotationalErrorThreshold);
        SmartDashboard.getNumber("Rotational Lower P", rotationalLowerP);
        SmartDashboard.getNumber("Rotational Lower P Threshold", rotationalLowerPThreshold);

    }

    @Override
    public void execute() {
        
        rotationalError = drivetrain.getHeadingBlue() - tagAngle;

        if (Math.abs(rotationalError) > rotationalLowerPThreshold) {
            rotationalPIDController.setP(rotationalP);
        } else {
            rotationalPIDController.setP(rotationalLowerP);
        }

        rotationalPIDController.setD(rotationalD);
        rotationalPIDController.setI(rotationalI);

        double rotation = 0;

        if(Math.abs(rotationalError) > rotationalErrorThreshold) {
            rotation = rotationalPIDController.calculate(rotationalError) + Math.signum(rotationalError) * rotationalFF;
        }

        drivetrain.drive(new Translation2d(0, 0), rotation, true, null);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
