// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.CollectBalls;
import frc.robot.commands.drivetrain.RotateToPose;
import frc.robot.commands.shooter.EjectOneBallBottom;
import frc.robot.commands.shooter.EjectOneBallTop;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallSteal extends SequentialCommandGroup {
  /** Creates a new TwoBallAndDAuto. */
  public TwoBallSteal(final XboxController controller, final DrivetrainSubsystem drivetrainSubsystem, final CollectorSubsystem collectorSubsystem, final StorageSubsystem storageSubsystem, final ShooterSubsystem shooterSubsystem, final LimeLight limelight, final RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //2BallAndDPart1
    addCommands(
      new OneBallAndD(controller, drivetrainSubsystem, collectorSubsystem, storageSubsystem, shooterSubsystem, limelight, robotContainer),
      new RotateToPose(drivetrainSubsystem, () -> Rotation2d.fromDegrees(-162.5).getRadians()),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(.5),
          drivetrainSubsystem.followPathCommand(false, "2BallStealPart3")
        ),
        new SequentialCommandGroup(
          new EjectOneBallBottom(storageSubsystem, collectorSubsystem),
          new CollectBalls(controller, collectorSubsystem, storageSubsystem, 2).withTimeout(1.7)
        )
      ),
      new InstantCommand(() -> storageSubsystem.setBallCount(2)),
      drivetrainSubsystem.followPathCommand(false, "2BallStealPart4"),
      new EjectOneBallTop(storageSubsystem, shooterSubsystem).withTimeout(2),
      new InstantCommand(() -> storageSubsystem.setBallCount(1)),
      robotContainer.shootCommandHelper(),
      new InstantCommand(() -> storageSubsystem.setBallCount(0))
      // need to spit the ball to destage
      
    );
  }
}
