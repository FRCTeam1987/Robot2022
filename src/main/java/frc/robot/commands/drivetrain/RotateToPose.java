// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// TODO doesn't work quite yet

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToPose extends PIDCommand {
  /** Creates a new RotateToAngle. */
  private final static PIDController m_PIDController = new PIDController(2.0, 0.0, 0);
  public RotateToPose(final DrivetrainSubsystem drivetrainSubsystem, final DoubleSupplier angleSupplier) {
    super(
        // The controller that the command will use
        // new PIDController(2.0, 0.0, 0),
        m_PIDController,
        // This should return the measurement
        () -> drivetrainSubsystem.getPose().getRotation().getRadians(),
        // This should return the setpoint (can also be a constant)
          () -> angleSupplier.getAsDouble(),
        // This uses the output
        output -> {
          drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, output, drivetrainSubsystem.getAdjustedHeading()));
          // System.out.println("The Heading is: " + drivetrainSubsystem.getAdjustedHeading().getDegrees());
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(Rotation2d.fromDegrees(2).getRadians(), 0.25);
    getController().enableContinuousInput(-Math.PI, Math.PI);
    drivetrainSubsystem.addChild("RotateToPose PID", m_PIDController);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
