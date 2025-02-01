package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.commands.FollowPathWithEvents;

//import frc.robot.commands.DriveCommands;

public class Aquamarine {
    
    public static Command driveByTime(CommandSwerveDrivetrain drivetrain, com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric drive){
        
        return Commands.sequence(Commands.print("driveByTime started"),
            Commands.print("applyRequest starting"),
            drivetrain.applyRequest(
                () -> {
                    System.out.print("drive.withVelocityX starting");
                    return (SwerveRequest) drive.withVelocityX(0.0);
                }
            ).withTimeout(5),
            drivetrain.runOnce(
                () -> {
                    drive.withVelocityX(0.0);
                }
            ),
            Commands.print("wait starting"),
            Commands.waitSeconds(5),
            Commands.print("wait ended"),
            drivetrain.applyRequest(
                () -> {
                    System.out.print("-drive.withVelocityY starting");
                    return (SwerveRequest) drive.withVelocityY(0.0);
                }
            ).withTimeout(5),
            drivetrain.runOnce(
                () -> {
                    drive.withVelocityY(0.0);
                }
            ),
            Commands.waitSeconds(5),
             drivetrain.applyRequest(() -> {
                    System.out.print("-drive.withVelocityX starting");
                    return (SwerveRequest) drive.withVelocityX(0.0);
                }
            ).withTimeout(5),
            drivetrain.runOnce(
                () -> {
                    drive.withVelocityX(0.0);
                }
            ),
            Commands.waitSeconds(5),
            drivetrain.run(
                () -> {
                    System.out.print("drive.withVelocityY starting");
                    drive.withVelocityY(0.0);
                }
            ).withTimeout(5),
            drivetrain.runOnce(
                () -> {
                    System.out.println("drive back to 0.0");
                    drive.withVelocityY(0.0);
                }
            ),
            Commands.print("withVelocity back to 0.0")
        );
    }

    
}
