// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.DriverOI;
import frc.robot.utils.DriverOI.DPadDirection;

public class AutoDriveCommand extends Command {
    private Drivetrain drivetrain;
    private Command pathCommand;

    public AutoDriveCommand(List<Pose2d> poseList, PathConstraints constraints, IdealStartingState start, GoalEndState end) {
        drivetrain = Drivetrain.getInstance();

        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            poseList
            // new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
            // new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(0))
            // new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            // new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );

        //PathConstraints constraints = new PathConstraints(3.0, 4.0, Math.PI, Math.PI); // The constraints for this path.
        // PathConstraints constraints = new PathConstraints(1.0, 1.0, Math.PI, Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, start, end);

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = false;

        pathCommand = AutoBuilder.followPath(path);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // drivetrain.drive(new Translation2d(0, 0), 0, true, new Translation2d(0, 0));
        pathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }
}
