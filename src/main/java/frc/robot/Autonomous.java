package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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
                new PIDConstants(1, 0, 0),
                new PIDConstants(1, 0, 0)
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
                Drivetrain.getInstance().setStartingPose(new Translation2d(0, 0));
            }),
            new AutoCommand()
            //new ShootFuelCommand()
            //new PassTrenchCommand(),

        );

        // Command depotOutPostCommand = new SequentialCommandGroup(
        //     new InstantCommand(() -> {
        //         Drivetrain.getInstance().setStartingPose(new Translation2d(2.512, 0.794));
        //     }),
        //     new AutoDriveCommand(new List(
        //         (new Pose2d(0.597, 0.664, Rotation2d.fromDegrees(12.875))), 
        //         (new Pose2d(2.564, 5.374, Rotation2d.fromDegrees(0))),
        //         (new Pose2d(0.779, 6.021, Rotation2d.fromDegrees(0)))), 
        //         new PathConstraints(3.0, 4.0, Math.PI, Math.PI)
        //     )
        // );


        autoChooser.setDefaultOption("1AutoCommand", autoCommand);

        SmartDashboard.putData("Auto Routines", autoChooser);

        autoStartPosition = new SendableChooser<>();
        autoStartPosition.setDefaultOption("1anywhere", 0.0);

        SmartDashboard.putData("Auto Starting Position", autoStartPosition);
    }

    public Command getAutonomousCommand(){
        Drivetrain.getInstance().setStartingPose(new Translation2d(0, 0));
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
            new Pose2d(1, 0.0, Rotation2d.fromDegrees(0))
            // new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            // new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );

        PathConstraints constraints = new PathConstraints(1.0, 4.0, Math.PI, Math.PI); // The constraints for this path.
        // PathConstraints constraints = new PathConstraints(1.0, 1.0, Math.PI, Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(0, new Rotation2d(0)), // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, new Rotation2d(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        List<PathPoint> points = path.getAllPathPoints();

        System.out.println(points.size());
        for (PathPoint p : points) {
            System.out.println(p.distanceAlongPath);
            var t2d = p.position;
            System.out.println("" + t2d.getX() + " " + t2d.getY());
        }

        // var traj = path.generateTrajectory(new ChassisSpeeds(0, 0, 0), new Rotation2d(0), config);
        // System.out.println("time = " + traj.getTotalTimeSeconds() + ", states = " + traj.getStates().size());

        Command com = AutoBuilder.followPath(path);

        return com;

        // return autoChooser.getSelected();
    }

    public double getStartHeading() {
        return 0;
        // return autoStartPosition.getSelected();
    }
}