// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CollectBalls;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuto extends SequentialCommandGroup {
  /** Creates a new FiveBallAuto. */
  public FiveBallAuto(final DrivetrainSubsystem drivetrainSubsystem, final CollectorSubsystem collectorSubsystem, final StorageSubsystem storageSubsystem, final ShooterSubsystem shooterSubsystem, final LimeLight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ThreeBallAuto(drivetrainSubsystem, collectorSubsystem, storageSubsystem, shooterSubsystem, limelight),
      new ParallelCommandGroup(
        drivetrainSubsystem.followPathCommand(false, "5BallAutoPart3B"),
        new SequentialCommandGroup(
          new WaitCommand(0.75), 
          new CollectBalls(collectorSubsystem, storageSubsystem, 2)
        )
      ),
      drivetrainSubsystem.followPathCommand(false, "5BallAutoPart4B"),
      new Shoot(shooterSubsystem, storageSubsystem, drivetrainSubsystem, limelight, () -> 2500, () -> 35).withTimeout(3)
    );
  }
}
