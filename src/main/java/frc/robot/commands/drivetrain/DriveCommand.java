// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier isFieldRelativeSupplier;
    private final SlewRateLimiter m_translationXLimiter;
    private final SlewRateLimiter m_translationYLimiter;
    private final SlewRateLimiter m_rotationLimiter;

    public DriveCommand(
            DrivetrainSubsystem drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier isFieldRelativeSupplier
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.isFieldRelativeSupplier = isFieldRelativeSupplier;
        m_translationXLimiter = new SlewRateLimiter(6);
        m_translationYLimiter = new SlewRateLimiter(6);
        m_rotationLimiter = new SlewRateLimiter(6);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // double translationXPercent = translationXSupplier.getAsDouble();
        // double translationYPercent = translationYSupplier.getAsDouble();
        // double rotationPercent = rotationSupplier.getAsDouble();
        double translationXPercent = m_translationXLimiter.calculate(translationXSupplier.getAsDouble());
        double translationYPercent = m_translationYLimiter.calculate(translationYSupplier.getAsDouble());
        double rotationPercent = m_rotationLimiter.calculate(rotationSupplier.getAsDouble());

        if (isFieldRelativeSupplier.getAsBoolean()) {
            drivetrain.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            translationXPercent * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                            translationYPercent * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                            rotationPercent * Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                            drivetrain.getAdjustedHeading()
                    )
            );
        } else {
            drivetrain.drive(new ChassisSpeeds(
                translationXPercent * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                translationYPercent * Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                rotationPercent * (Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .8)
            ));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
