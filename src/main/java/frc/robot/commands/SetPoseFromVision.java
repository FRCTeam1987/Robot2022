// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPoseFromVision extends InstantCommand {

  private final Translation2d k_goalPosition = new Translation2d(8.23, 4.11);
  private static final double k_limelightMountAngleDegrees = 39.0;
  private static final double k_goalHeightMeters = Units.inchesToMeters(104);
  private static final double k_limelightMountHeightMeters = Units.inchesToMeters(28);
  private static final double radiusOfUpperHub = 0.68; //meters
  private static final double distanceFromLimelightToCenterOfRobot = Units.inchesToMeters(9);

  private final DrivetrainSubsystem m_drive;
  private final LimeLight m_limeLight;

  

  public SetPoseFromVision(final DrivetrainSubsystem drive, final LimeLight limeLight) {
    m_drive = drive;
    m_limeLight = limeLight;
    ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drivetrain");
    driveTrainTab.addNumber("distance to Goal", this::getDistanceToHub);
    driveTrainTab.addNumber("Ty To Goals", this::calculateTyToGoal);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final Rotation2d robotAngle = m_drive.getAdjustedHeading();
    final double distanceToGoal = calculateDistanceToGoal(m_limeLight.getYAxis()) + radiusOfUpperHub;
    m_drive.setPose(new Pose2d(
      k_goalPosition.plus(new Translation2d(distanceToGoal, robotAngle)),
      robotAngle
    ));
    SmartDashboard.putNumber("Distance To Goals", distanceToGoal);
    // SmartDashboard.putNumber("Pose Y", m_drive.getPoseMeters().getY());
  }

  private double calculateDistanceToGoal(final double ty) {
    return (k_goalHeightMeters - k_limelightMountHeightMeters) / Math.tan(Units.degreesToRadians(k_limelightMountAngleDegrees + ty));
  }

  private double calculateTyToGoal() {
    // return Math.atan(()/m_limeLight.)
    return Math.atan((k_goalHeightMeters - k_limelightMountHeightMeters) / getDistanceToHub()) - k_limelightMountAngleDegrees;
    // return 0.0;
  }

  private double getDistanceToHub() {
    return k_goalPosition.getDistance(m_drive.getPose().getTranslation());

    // return 0.0;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
