// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetDistanceFromHub extends InstantCommand {
  private DrivetrainSubsystem  m_driveTrain;
  private LimeLight m_limelight;

  public GetDistanceFromHub(DrivetrainSubsystem driveTrain, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_limelight = limeLight;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Math.atan(
      m_limelight.getYAxis()/
      m_limelight.getXAxis()
    );

  }
}
