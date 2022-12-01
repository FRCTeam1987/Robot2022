// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.RotateToLimelightAngle;
import frc.robot.commands.storage.FeedShooterWithRPM;
import frc.robot.commands.storage.StopStorage;
import frc.robot.commands.storage.WaitForBallShoot;
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
  private final Compressor m_compressor;

  /** Creates a new ShooterCommandGroup. */
  public Shoot(final ShooterSubsystem shooter, final StorageSubsystem storage, final DrivetrainSubsystem drivetrain, final LimeLight limelight, final Compressor compressor, final DoubleSupplier rpmSupplier, final BooleanSupplier isHoodRaised) {
    m_shooter = shooter;
    m_storage = storage;
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    m_compressor = compressor;
    
    addCommands(
      new ParallelCommandGroup(
        new ConditionalCommand(
          new InstantCommand(() -> m_storage.runForIntake(), m_storage),
          new InstantCommand(),
          () -> m_storage.getBallCount() == 1
        ),
        new InstantCommand(() -> m_shooter.setRPM(Constants.Shooter.PRE_SHOOT_RPM), m_shooter)
      ),
      new RotateToLimelightAngle(m_drivetrain, m_limelight),
      new ConditionalCommand(
        new RaiseHood(m_shooter),
        new LowerHood(m_shooter),
        isHoodRaised
      ),
      new InstantCommand(() -> m_compressor.disable()),
      new SetShooterRpm(m_shooter, rpmSupplier),
      // new SetHoodPosition(m_shooter, hoodPos),
      
      
      // if (hoodPos > 57.5) {
      //   new HoodHighPosition(m_shooter);
      // } else {
      //   new HoodLowPosition(m_shooter);
      // }
      // () -> hoodPos == true ? new HoodHighPosition(m_shooter) : new HoodLowPosition(m_shooter),

      new ParallelRaceGroup(
        new WaitForBallShoot(m_storage),
        new FeedShooterWithRPM(m_storage, m_shooter)
      ),
      new StopStorage(m_storage),
      new SetShooterRpm(m_shooter, () -> 0)
    );
  }

  public Shoot(final ShooterSubsystem shooter, final StorageSubsystem storage, final DrivetrainSubsystem drivetrain, final LimeLight limelight, final Compressor compressor) {
    this(
      shooter,
      storage,
      drivetrain,
      limelight,
      compressor,
      () -> 2225,
      () -> true 
    );
  }


  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (interrupted) {
      m_shooter.stop();
      m_storage.stop();
    }
    m_compressor.enableAnalog(100, 120);
  }
}
