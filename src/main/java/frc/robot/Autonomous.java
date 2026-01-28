package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToHPBasisVector;
import frc.robot.commands.AlignToReefBasisVector;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
import frc.robot.utils.Constants.AlignmentConstants.HPAlign;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlign;
import frc.robot.utils.Logger;

public class Autonomous {

    private static Autonomous autonomous;
    private SendableChooser<Command> autoChooser;
    private SendableChooser<Double> autoStartPosition;

    public static Autonomous getInstance() {
        if (autonomous == null)
            autonomous = new Autonomous();
        return autonomous;
    }
    RobotConfig config;

    private void startSnakeDrive() {
        PPHolonomicDriveController.overrideRotationFeedback(() -> {
            return Drivetrain.getInstance().getRotationOverride();
        });
    }
    private void stopSnakeDrive() {
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    private Autonomous() {
        Drivetrain drivetrain = Drivetrain.getInstance();

        // config = new RobotConfig(
        //     74.088,
        //     6.883,
        //     new ModuleConfig(0.048, 5.450, 1.200, DCMotor.getKrakenX60(1), 60, 1),
        //     new Translation2d(0.273, 0.273),
        //     new Translation2d(0.273, -0.273),
        //     new Translation2d(-0.273, 0.273),
        //     new Translation2d(-0.273, -0.273)
        // );

        try {
            config = RobotConfig.fromGUISettings();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            drivetrain::getPose,
            drivetrain::setPose,
            drivetrain::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> drivetrain.driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0),
                new PIDConstants(5, 0, 0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            drivetrain // Reference to this subsystem to set requirements
        );



        autoChooser = new SendableChooser<>();

        

        Command autoCommand = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(1, 0, Rotation2d.fromDegrees(180)));
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(1, 0, Rotation2d.fromDegrees(180)), 
                    new Pose2d(0, 0, Rotation2d.fromDegrees(180))
                ),
                new PathConstraints(0.5, 0.5, Math.PI, Math.PI),
                // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                new GoalEndState(0, Rotation2d.fromDegrees(0))
            )

        );

        Command rightOutDepClimbLAuto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)));
                startSnakeDrive();
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)), 
                    new Pose2d(1.257, 0.639, Rotation2d.fromDegrees(180.000)),
                    new Pose2d(2.380, 3.729, Rotation2d.fromDegrees(61.557)),
                    new Pose2d(1.257, 5.882, Rotation2d.fromDegrees(156.038)),
                    new Pose2d(1.069, 4.702, Rotation2d.fromDegrees(-59.534))
                ),
                List.of(
                    new EventMarker("Stop snake drive", 2.42, new InstantCommand(() -> stopSnakeDrive()))
                ),
                new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                new GoalEndState(0, Rotation2d.fromDegrees(0))
            )

        );


        Command leftDepOutClimbRAuto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(3.560, 5.853, Rotation2d.fromDegrees(180)));
                //startSnakeDrive();
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(3.560, 5.853, Rotation2d.fromDegrees(180)), 
                    new Pose2d(0.960, 6.008, Rotation2d.fromDegrees(-159.829)),
                    new Pose2d(2.448, 3.550, Rotation2d.fromDegrees(-83.367)),
                    new Pose2d(0.791, 1.053, Rotation2d.fromDegrees(92.651)),
                    new Pose2d(1.076, 3.162, Rotation2d.fromDegrees(95.631))
                ),
                new PathConstraints(1.5, 1.5, 3 * Math.PI, 4 * Math.PI),

                //new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                new GoalEndState(0, Rotation2d.fromDegrees(180))
            )

        );

        Command crazyWeirdAuto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(180)));
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(124.306)), 
                    new Pose2d(2.034, 2.541, Rotation2d.fromDegrees(91.449)),
                    new Pose2d(2.771, 3.912, Rotation2d.fromDegrees(81.251)),
                    new Pose2d(2.304, 5.297, Rotation2d.fromDegrees(77.259))
                ),
                List.of(
                    //add rotation target / holonomic rotations code!!!
                ),
                new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                new GoalEndState(0, Rotation2d.fromDegrees(-90))
            )

        );


        // Command rightOutDepClimbAuto = new SequentialCommandGroup(
        //     new InstantCommand(() -> {
        //         Drivetrain.getInstance().setStartingPose(new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)));
        //         startSnakeDrive();
        //     }),
        //     new AutoDriveCommand(
        //         List.of(
        //             new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)), 
        //             new Pose2d(1.257, 0.639, Rotation2d.fromDegrees(180.000)),
        //             new Pose2d(2.380, 3.729, Rotation2d.fromDegrees(61.557)),
        //             new Pose2d(1.257, 5.882, Rotation2d.fromDegrees(156.038)),
        //             new Pose2d(1.069, 4.702, Rotation2d.fromDegrees(-59.534))
        //         ),
        //         List.of(
        //             new EventMarker("Stop snake drive", 2.42, new InstantCommand(() -> stopSnakeDrive()))
        //         ),
        //         new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
        //         // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
        //         new IdealStartingState(0, Rotation2d.fromDegrees(180)),
        //         new GoalEndState(0, Rotation2d.fromDegrees(180))
        //     )
        // );


        

        // Command outpostAuto = new SequentialCommandGroup(
        //     new InstantCommand(() -> {
        //         Drivetrain.getInstance().setStartingPose(new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(-90)));
        //     }),
        //     new AutoDriveCommand(
        //         List.of(
        //             new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(0)), 
        //             new Pose2d(0.584,0.664, Rotation2d.fromDegrees(0))
        //         ),
        //         new PathConstraints(2, 4, Math.PI, Math.PI),
        //         // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
        //         new IdealStartingState(0, Rotation2d.fromDegrees(0)),
        //         new GoalEndState(0, Rotation2d.fromDegrees(0))
        //     )
        // );


        // Command notsosquiggleAuto = new SequentialCommandGroup(
        //     new InstantCommand(() -> {
        //         Drivetrain.getInstance().setStartingPose(new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(180)));
        //         startSnakeDrive();
        //     }),
        //     new AutoDriveCommand(
        //         List.of(
        //             new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(175.355)), 
        //             new Pose2d(2.202,2.735, Rotation2d.fromDegrees(100.909)),
        //             new Pose2d(2.034,5.297, Rotation2d.fromDegrees(90))
        //         ),
        //         new PathConstraints(1, 1, 6 * Math.PI, 8 * Math.PI),
        //         // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
        //         new IdealStartingState(0, Rotation2d.fromDegrees(0)),
        //         new GoalEndState(0, Rotation2d.fromDegrees(0))
        //     )
        // );

        // Command squiggleAuto = new SequentialCommandGroup(
        //     new InstantCommand(() -> {
        //         Drivetrain.getInstance().setStartingPose(new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(180)));
        //         startSnakeDrive();
        //     }),
        //     new AutoDriveCommand(
        //         List.of(
        //             new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(149.577)), 
        //             new Pose2d(2.034,2.541, Rotation2d.fromDegrees(42.614)),
        //             new Pose2d(2.771,3.912, Rotation2d.fromDegrees(81.251)),
        //             new Pose2d(2.034,5.297, Rotation2d.fromDegrees(77.259))
        //         ),
        //         new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
        //         // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
        //         new IdealStartingState(0, Rotation2d.fromDegrees(0)),
        //         new GoalEndState(0, Rotation2d.fromDegrees(0))
        //     )
        // );


        autoChooser.setDefaultOption("1AutoCommand", autoCommand);
        autoChooser.setDefaultOption("R - Outpost/Depot/Left Climb Auto", rightOutDepClimbLAuto);
        autoChooser.setDefaultOption("L - Depost/Outpost/Right Climb Auto", leftDepOutClimbRAuto);
        // autoChooser.setDefaultOption("Outpost", outpostAuto);
        // autoChooser.setDefaultOption("Squiggle", squiggleAuto);
        // autoChooser.setDefaultOption("Not Squiggle", notsosquiggleAuto);

        SmartDashboard.putData("Auto Routines", autoChooser);

        autoStartPosition = new SendableChooser<>();
        autoStartPosition.setDefaultOption("1anywhere", 0.0);

        SmartDashboard.putData("Auto Starting Position", autoStartPosition);
    }

    public Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }
}