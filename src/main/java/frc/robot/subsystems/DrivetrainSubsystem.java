// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Drivetrain.*;

import java.util.Set;

public class DrivetrainSubsystem extends SubsystemBase {

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Front right
    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back left
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back right
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private final AHRS m_navx = new AHRS();
  private Rotation2d m_headingAdjust = new Rotation2d();

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private double m_pitchOffset;
  private Rotation2d m_startYaw;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAdjustedHeading());

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    m_startYaw = new Rotation2d();

    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4iSwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );
    zeroGyroscope();
    tab.addNumber("Adjusted Angle", () -> getAdjustedHeading().getDegrees()).withPosition(9, 2);
    tab.addNumber("Heading", () -> m_navx.getCompassHeading()).withPosition(9, 3);
    tab.addNumber("Yaw", () -> m_navx.getYaw()).withPosition(9, 1);
    // tab.addNumber("Heading Adjust", () -> getHeadingAdjust().getDegrees()).withPosition(8, 2);
    tab.addNumber("Pose X", () -> m_odometry.getPoseMeters().getX()).withPosition(8, 0);
    tab.addNumber("Pose Y", () -> m_odometry.getPoseMeters().getY()).withPosition(8, 1);
    tab.addNumber("Pose Angle", () -> m_odometry.getPoseMeters().getRotation().getDegrees()).withPosition(8, 2);
    tab.addNumber("Pitch", () -> getPitch()).withPosition(8, 3);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_navx.reset();
    // setHeadingAdjust(Rotation2d.fromDegrees(m_navx.getCompassHeading()));
    // System.out.println("Zeroing gyro: " + m_navx.getCompassHeading());
  }
  public void rememberStartingPosition() {
    m_startYaw = Rotation2d.fromDegrees(m_navx.getYaw());
  }

  public void reZeroFromStartingPositon() {
    setHeadingAdjust(Rotation2d.fromDegrees(m_navx.getCompassHeading()).minus(m_startYaw));
  }

  public Rotation2d getHeadingAdjust() {
    return m_headingAdjust;
  }

  private void setHeadingAdjust(final Rotation2d headingAdjust) {
    m_headingAdjust = headingAdjust;
    // System.out.println("Set heading adjust: " + m_headingAdjust.getDegrees());
  }

  private Rotation2d getUnadjustedHeading() {
    return Rotation2d.fromDegrees(m_navx.getCompassHeading());
  }

  public Rotation2d getAdjustedHeading() {
    // TODO compass heading only returns 180 now... why?
  //  if (m_navx.isMagnetometerCalibrated()) {
     // We will only get valid fused headings if the magnetometer is calibrated
    //  return Rotation2d.fromDegrees(m_navx.getCompassHeading()).rotateBy(getHeadingAdjust());
  //  }

   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  /* reused code implementation is pain */
  public Rotation2d getYaw() {
    return getAdjustedHeading();
  }

  public double getPitch() {
    return m_navx.getPitch();
  }

  private void setPitchOffset(double pitchOffset) {
    m_pitchOffset = pitchOffset;
  }

  public double getPitchOffset() {
    return m_pitchOffset;
  }

  public double getPitchWithOffset() {
    return getPitch() + getPitchOffset();
  }

  public void zeroPitch() {
    setPitchOffset(-getPitch());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void driveVolts(final double volts, final boolean isRotation) {
    if (isRotation) { // TODO get these angles
      m_frontLeftModule.set(volts, -Math.toRadians(45));
      m_frontRightModule.set(volts, -Math.toRadians(45));
      m_backLeftModule.set(volts, -Math.toRadians(45));
      m_backRightModule.set(volts, -Math.toRadians(45));
    } else {
      m_frontLeftModule.set(volts, -Math.toRadians(0));
      m_frontRightModule.set(volts, -Math.toRadians(0));
      m_backLeftModule.set(volts, -Math.toRadians(0));
      m_backRightModule.set(volts, -Math.toRadians(0));
    }
  }

  public double getVelocityFL() {
    return m_frontLeftModule.getDriveVelocity();
  }

  public double getVelocityFR() {
    return m_frontRightModule.getDriveVelocity();
  }

  public double getVelocityBL() {
    return m_backLeftModule.getDriveVelocity();
  }

  public double getVelocityBR() {
    return m_backRightModule.getDriveVelocity();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Pick back up here with path following constant placeholders
  public SequentialCommandGroup followPathCommand(final boolean shouldResetOdometry, String trajectoryFileName) {
    final PIDController xController = new PIDController(DRIVETRAIN_PX_CONTROLLER, 0, 0);
    final PIDController yController = new PIDController(DRIVETRAIN_PY_CONTROLLER, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(DRIVETRAIN_PTHETA_CONTROLLER, 0, 0, new TrapezoidProfile.Constraints(
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_SQUARED
    ));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // final Trajectory trajectory = generateTrajectory(waypoints);
    final PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFileName, 3.5, 3   );
    // double Seconds = 0.0;
    // System.out.println("===== Begin Sampling path =====");
    // while(trajectory.getTotalTimeSeconds() > Seconds) {
    //   PathPlannerState state = (PathPlannerState) trajectory.sample(Seconds);
    //   System.out.println(
    //     "time: " + Seconds
    //     + ", x: " + state.poseMeters.getX()
    //     + ", y: " + state.poseMeters.getY()
    //     + ", angle: " + state.poseMeters.getRotation().getDegrees()
    //     + ", holo: " + state.holonomicRotation.getDegrees()
    //   );
    // Seconds += 0.25;
    // }
    // System.out.println("===== End Sampling Path =====");
    return new InstantCommand(() -> {
      if (shouldResetOdometry) {
        PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
        Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(), initialSample.holonomicRotation);
        m_odometry.resetPosition(initialPose, getAdjustedHeading());
      }
    }).andThen(new PPSwerveControllerCommand(
      trajectory,
      () -> getPose(),
      m_kinematics,
      xController,
      yController,
      thetaController,
      (SwerveModuleState[] moduleStates) -> {
        drive(m_kinematics.toChassisSpeeds(moduleStates));
      },
      this
    )).andThen(() -> drive(new ChassisSpeeds()), this);
  }

  public double velocityToVolts(final double desiredVelocity) {
    return MathUtil.clamp(TRANSLATION_FEED_FORWARD.calculate(desiredVelocity), -MAX_VOLTAGE, MAX_VOLTAGE);
  }

  @Override
  public void periodic() {
    m_odometry.update(
      getAdjustedHeading(),
      new SwerveModuleState(m_frontLeftModule.getDriveVelocity(), new Rotation2d(m_frontLeftModule.getSteerAngle())),
      new SwerveModuleState(m_frontRightModule.getDriveVelocity(), new Rotation2d(m_frontRightModule.getSteerAngle())),
      new SwerveModuleState(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle())),
      new SwerveModuleState(m_backRightModule.getDriveVelocity(), new Rotation2d(m_backRightModule.getSteerAngle()))
    );

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    // m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    // m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    // m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    // m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    m_frontLeftModule.set(velocityToVolts(states[0].speedMetersPerSecond), states[0].angle.getRadians());
    m_frontRightModule.set(velocityToVolts(states[1].speedMetersPerSecond), states[1].angle.getRadians());
    m_backLeftModule.set(velocityToVolts(states[2].speedMetersPerSecond), states[2].angle.getRadians());
    m_backRightModule.set(velocityToVolts(states[3].speedMetersPerSecond), states[3].angle.getRadians());
    if (!m_navx.isConnected()) {
      DriverStation.reportError("navx not connected", false);
    }
  }
}