// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drivetrain.RotateToLimelightAngle;
import frc.robot.commands.storage.FeedShooter;
import frc.robot.commands.storage.FeedShooterWithRPM;
import frc.robot.commands.storage.StopStorage;
import frc.robot.commands.storage.WaitForBallShoot;
import frc.robot.lib.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {

  private final ShooterSubsystem m_shooter;
  private final StorageSubsystem m_storage;
  private final DrivetrainSubsystem m_drivetrain;
  private final LimeLight m_limelight;

  /** Creates a new ShooterCommandGroup. */
  public Shoot(final ShooterSubsystem shooter, final StorageSubsystem storage, final DrivetrainSubsystem drivetrain, final LimeLight limelight, final DoubleSupplier rpmSupplier, final DoubleSupplier hoodSupplier) {
    m_shooter = shooter;
    m_storage = storage;
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    
    addCommands(
      // new ParallelCommandGroup(
      new RotateToLimelightAngle(m_drivetrain, limelight),
      new SetShooterRpm(m_shooter, rpmSupplier),
      new SetHoodPosition(m_shooter, hoodSupplier),
      // ),
      // new WaitUntilCommand(0.25), // Don't want to have this, but may 
      // new FeedShooter(m_storage),
      new ParallelRaceGroup(
        new WaitForBallShoot(m_storage),
        new FeedShooterWithRPM(m_storage, m_shooter)
      ),
      // new WaitCommand(0.25),
      new StopStorage(m_storage),
      new SetShooterRpm(m_shooter, () -> 0)
    );
  }

  public Shoot(final ShooterSubsystem shooter, final StorageSubsystem storage, final DrivetrainSubsystem drivetrain, final LimeLight limelight) {
    this(
      shooter,
      storage,
      drivetrain,
      limelight,
      () -> SmartDashboard.getNumber("RPM-Set", 0.0),
      () -> SmartDashboard.getNumber("Hood-Pos", 0.0)
    );
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    if (interrupted) {
      m_shooter.stop();
      m_storage.stop();
      m_storage.zeroBallCount();
    }
  }
}
