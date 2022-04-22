// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.CollectBalls;
import frc.robot.commands.collector.DeployCollector;
import frc.robot.commands.collector.StowCollector;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.storage.SetBallCount;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new FiveBallAuto. */
  public ThreeBallAuto(final XboxController controller, final DrivetrainSubsystem drivetrainSubsystem, final CollectorSubsystem collectorSubsystem, final StorageSubsystem storageSubsystem, final ShooterSubsystem shooterSubsystem, final LimeLight limelight, final RobotContainer robotContainer) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        drivetrainSubsystem.followPathCommand(true, "3BallAutoPart1"),
        new CollectBalls(controller, collectorSubsystem, storageSubsystem, 2)
      ),
      new InstantCommand(() -> DriverStation.reportWarning("Three Ball - A", false)),
      robotContainer.shootCommandHelper(),
      new InstantCommand(() -> DriverStation.reportWarning("Three Ball - B", false)),
      new ParallelCommandGroup(
        drivetrainSubsystem.followPathCommand(false, "3BallAutoPart2"),
        new SequentialCommandGroup(
          new WaitCommand(0.25), 
          new CollectBalls(controller, collectorSubsystem, storageSubsystem, 1)
        )
      ),
      new InstantCommand(() -> DriverStation.reportWarning("Three Ball - C", false)),
      // new InstantCommand(() -> storageSubsystem.runForIntake(), storageSubsystem),
      robotContainer.shootCommandHelper(),
      new InstantCommand(() -> DriverStation.reportWarning("Three Ball - D", false))
    );
  }
}
