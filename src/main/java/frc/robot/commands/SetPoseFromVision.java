// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPoseFromVision extends InstantCommand {

  private static final Translation2d k_goalPosition = new Translation2d(8.23, 4.11);
  private static final double k_limelightMountAngleDegrees = 39.0;
  private static final double k_goalHeightMeters = 0.0; // TODO
  private static final double k_limelightMountHeightMeters = 0.0;

  private final DrivetrainSubsystem m_drive;
  private final LimeLight m_limeLight;

  public SetPoseFromVision(final DrivetrainSubsystem drive, final LimeLight limeLight) {
    m_drive = drive;
    m_limeLight = limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final Rotation2d robotAngle = m_drive.getAdjustedHeading();
    final double distanceToGoal = calculateDistanceToGoal(m_limeLight.getYAxis());
    m_drive.setPose(new Pose2d(
      k_goalPosition.plus(new Translation2d(distanceToGoal, robotAngle)),
      robotAngle
    ));
  }

  private double calculateDistanceToGoal(final double ty) {
    return (k_goalHeightMeters - k_limelightMountHeightMeters) / Math.tan(Units.degreesToRadians(k_limelightMountAngleDegrees + m_limeLight.getYAxis()));
  }
}
