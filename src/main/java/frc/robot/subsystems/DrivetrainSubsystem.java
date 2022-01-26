// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.drivetrain.PathPlannerSwerveControllerCommand;
import frc.robot.lib.pathplannerlib.PathPlanner;
import frc.robot.lib.pathplannerlib.PathPlannerTrajectory;
import frc.robot.lib.pathplannerlib.PathPlannerTrajectory.PathPlannerState;

import static frc.robot.Constants.*;

import java.util.List;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = FALCON_MAX_RPM / 60.0 *
          SdsModuleConfigurations.MK3_FAST.getDriveReduction() *
          SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /  // TODO update this from max auto velocity
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_SQUARED = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 3.0;  

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

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final AHRS m_navx = new AHRS();
  private Rotation2d m_headingAdjust = new Rotation2d();

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAdjustedHeading());
  private final TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(3, 3).setKinematics(m_kinematics); // TODO determine good auto max speed / acceleration

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk3SwerveModuleHelper.GearRatio.FAST,
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
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
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
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_navx.reset();
    setHeadingAdjust(Rotation2d.fromDegrees(m_navx.getCompassHeading()));
    System.out.println("Zeroing gyro: " + m_navx.getCompassHeading());
  }

  public Rotation2d getHeadingAdjust() {
    return m_headingAdjust;
  }

  private void setHeadingAdjust(final Rotation2d headingAdjust) {
    m_headingAdjust = headingAdjust;
    System.out.println("Set heading adjust: " + m_headingAdjust.getDegrees());
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

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  private TrajectoryConfig getTrajectoryConfig() {
    return m_trajectoryConfig;
  }

  private Trajectory generateTrajectory(Pose2d... waypoints) {
    return TrajectoryGenerator.generateTrajectory(List.of(waypoints), getTrajectoryConfig());
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
    final PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFileName, 4, 3.5);
    double Seconds = 0.0;
    System.out.println("===== Begin Sampling path =====");
    while(trajectory.getTotalTimeSeconds() > Seconds) {
      PathPlannerState state = (PathPlannerState) trajectory.sample(Seconds);
      System.out.println(
        "time: " + Seconds
        + ", x: " + state.poseMeters.getX()
        + ", y: " + state.poseMeters.getY()
        + ", angle: " + state.poseMeters.getRotation().getDegrees()
        + ", holo: " + state.holonomicRotation.getDegrees()
      );
    Seconds += 0.25;
    }
    System.out.println("===== End Sampling Path =====");
    return new InstantCommand(() -> {
      if (shouldResetOdometry) {
        PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
        Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(), initialSample.holonomicRotation);
        m_odometry.resetPosition(initialPose, getAdjustedHeading());
      }
    }).andThen(new PathPlannerSwerveControllerCommand(
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

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }
}