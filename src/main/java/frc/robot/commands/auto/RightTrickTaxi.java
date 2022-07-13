// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.LimeLight;
import frc.robot.commands.CollectBalls;
import frc.robot.commands.drivetrain.RotateToAngle;
import frc.robot.commands.shooter.EjectOneBallBottom;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightTrickTaxi extends SequentialCommandGroup {
  /** Creates a new RightTrickTaxi. */
  public RightTrickTaxi(final XboxController controller, final DrivetrainSubsystem drivetrainSubsystem, final CollectorSubsystem collectorSubsystem, final StorageSubsystem storageSubsystem, final ShooterSubsystem shooterSubsystem, final frc.robot.subsystems.LimeLight m_limelight, final RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(2),
      robotContainer.shootCommandHelper(),
      new ParallelCommandGroup(
        drivetrainSubsystem.followPathCommand(true, "RightTrickTaxiPart1"),
        new SequentialCommandGroup(
          new WaitCommand(0.125),
          new CollectBalls(controller, collectorSubsystem, storageSubsystem, 1)
        )
      ),
      drivetrainSubsystem.followPathCommand(true, "RightTrickTaxiPart2"),
      new EjectOneBallBottom(storageSubsystem, collectorSubsystem),
      drivetrainSubsystem.followPathCommand(false, "1BallAndStealPart4")
      
      // new RotateToAngle(drivetrainSubsystem, Math.toDegrees(45))

    );
  }
}
