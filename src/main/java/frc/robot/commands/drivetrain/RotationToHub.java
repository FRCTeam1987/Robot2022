// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotationToHub extends RotateToAngle {
  public RotationToHub(DrivetrainSubsystem drivetrain) {
        super(drivetrain, () -> drivetrain.getPose().getX() > 8.25 ?
            (90 + Math.atan((drivetrain.getPose().getY() - Constants.Shooter.HubYPosition)/
            (drivetrain.getPose().getX() - Constants.Shooter.HubXPosition)))
            : (180 + Math.atan((drivetrain.getPose().getY() - Constants.Shooter.HubYPosition)/
            (drivetrain.getPose().getX() - Constants.Shooter.HubXPosition)))
        );
    }
}