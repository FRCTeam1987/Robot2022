// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;

public class RotateToLimelightAngle extends RotateToAngle {
  /** Creates a new RotateToLimelightAngle. */
  public RotateToLimelightAngle(final DrivetrainSubsystem drivetrain, final LimeLight limelight) {
    super(drivetrain, () -> Rotation2d.fromDegrees(limelight.getXAxis()).minus(drivetrain.getAdjustedHeading()).getRadians());
  }
}