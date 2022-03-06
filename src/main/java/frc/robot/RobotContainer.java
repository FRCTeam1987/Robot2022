// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ChangeLimeLightStream;
import frc.robot.commands.CollectBalls;
import frc.robot.commands.ChangeLimeLightStream.StreamType;
import frc.robot.commands.auto.FiveBallAuto;
import frc.robot.commands.collector.DeployCollector;
import frc.robot.commands.collector.StowCollector;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.RotateToAngle;
import frc.robot.commands.drivetrain.RotateToLimelightAngle;
import frc.robot.commands.drivetrain.SwerveCharacterizationFF;
import frc.robot.commands.shooter.EjectOneBallBottom;
import frc.robot.commands.shooter.EjectOneBallTop;
import frc.robot.commands.shooter.SetHoodPosition;
import frc.robot.commands.shooter.SetShooterRpm;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.storage.FeedShooter;
import frc.robot.commands.storage.RunStorageIn;
import frc.robot.commands.storage.RunStorageOut;
import frc.robot.commands.storage.SetBallCount;
import frc.robot.commands.storage.StopStorage;
import frc.robot.commands.storage.ZeroBallCount;
import frc.robot.lib.Limelight;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
  // private final UsbCamera m_camera = CameraServer.startAutomaticCapture();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final CollectorSubsystem m_collector = new CollectorSubsystem();
  private final StorageSubsystem m_storage = new StorageSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final LimeLight m_limelight = new LimeLight();

  private final XboxController controller = new XboxController(0);
  private final XboxController coController = new XboxController(1);
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  // private final Thread m_visionThread = new Thread(() -> {
  //   final UsbCamera camera = CameraServer.startAutomaticCapture();
  //   camera.setResolution(320, 240);
  //   camera.setFPS(30);
  // });

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    // drivetrain.register();
    // collector.register();
    // m_camera.setResolution(320, 240);
    // m_camera.setFPS(30);

    // m_visionThread.setDaemon(true);
    // m_visionThread.start();

    m_drivetrain.setDefaultCommand(new DriveCommand(
            m_drivetrain,
            () -> -modifyAxis(controller.getLeftY()), // Axes are flipped here on purpose
            () -> -modifyAxis(controller.getLeftX()),
            () -> -modifyAxis(controller.getRightX())
    ));

    configureButtonBindings();
    configureShuffleboard();
    m_compressor.enableAnalog(100, 120);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Button(controller::getBackButtonPressed)
      .whenPressed(new InstantCommand(() -> { m_drivetrain.zeroGyroscope(); }));
    new Button(controller::getRightBumperPressed)
      .whenPressed(new CollectBalls(m_collector, m_storage, 2));
    new Button(controller::getRightBumperReleased)
      .whenReleased(new StowCollector(m_collector)
        .andThen(new StopStorage(m_storage)));
    new Button(controller::getLeftBumper)
      .whileHeld(new Shoot(
        m_shooter,
        m_storage,
        m_drivetrain,
        m_limelight,
        () -> m_limelight.getYAxis() < -5 ? 3075 : 2500,
        () -> m_limelight.getYAxis() < -5 ? 70 : 35
      ));

    new Button(coController::getYButton)
      .whenPressed(new EjectOneBallTop(m_storage, m_shooter));
    new Button(coController::getAButton)
      .whenPressed(new EjectOneBallBottom(m_storage, m_collector));
    new POVButton(coController, 0)
      .whenPressed(new SetBallCount(m_storage, 2));
    new POVButton(coController, 90)
      .whenPressed(new SetBallCount(m_storage, 1));
    new POVButton(coController, 180)
      .whenPressed(new SetBallCount(m_storage, 0));
  };

  private void configureShuffleboard() {
    m_autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    m_autoChooser.addOption("5 Ball Auto", new FiveBallAuto(m_drivetrain, m_collector, m_storage, m_shooter, m_limelight));
    m_autoChooser.addOption("Swerve Char - Forwards", new SwerveCharacterizationFF(m_drivetrain, true, false));
    m_autoChooser.addOption("Swerve Char - Reverse", new SwerveCharacterizationFF(m_drivetrain, false, false));
    m_autoChooser.addOption("Swerve Char - Rotate", new SwerveCharacterizationFF(m_drivetrain, true, true));
    SmartDashboard.putData(m_autoChooser);

    SmartDashboard.putData("Collect 1 Balls", new CollectBalls(m_collector, m_storage, 1));
    SmartDashboard.putData("Collect 2 Balls", new CollectBalls(m_collector, m_storage, 2));
    SmartDashboard.putData("Collector-Deploy", new DeployCollector(m_collector));
    SmartDashboard.putData("Collector-Stow", new StowCollector(m_collector));
    SmartDashboard.putData("Feed Shooter", new FeedShooter(m_storage));
    SmartDashboard.putData("Set Hood Pos",new SetHoodPosition(m_shooter, () -> SmartDashboard.getNumber("Hood-Pos", 0.0)));
    // SmartDashboard.putData("Shooter-Spin", new SetShooterRpm(m_shooter, () -> 1500.0));
    // SmartDashboard.putData("Shooter-Stop", new SetShooterRpm(m_shooter, () -> 0.0));
    SmartDashboard.putData("Store-In", new RunStorageIn(m_storage));
    SmartDashboard.putData("Store-Out", new RunStorageOut(m_storage));
    SmartDashboard.putData("Store-Stop", new StopStorage(m_storage));
    SmartDashboard.putData("LL-Standard", new ChangeLimeLightStream(StreamType.standard));
    SmartDashboard.putData("LL-PipMain", new ChangeLimeLightStream(StreamType.pipMain));
    SmartDashboard.putData("LL-PipSecondary", new ChangeLimeLightStream(StreamType.pipSecondary));
    SmartDashboard.putData("shoot", new Shoot(m_shooter, m_storage, m_drivetrain, m_limelight));
    SmartDashboard.putData("Reset Ball Count", new ZeroBallCount(m_storage));
    SmartDashboard.putData("Rotate-45", new RotateToAngle(m_drivetrain, () -> Rotation2d.fromDegrees(45).getRadians()));
    SmartDashboard.putData("Rotate-LL", new RotateToLimelightAngle(m_drivetrain, m_limelight));
  }

  private static double deadband(double value, double deadband) {
      if (Math.abs(value) > deadband) {
          if (value > 0.0) {
              return (value - deadband) / (1.0 - deadband);
          } else {
              return (value + deadband) / (1.0 - deadband);
          }
      } else {
          return 0.0;
      }
  }

  private static double modifyAxis(double value) {
      // Deadband
      value = deadband(value, 0.05);

      // Square the axis
      value = Math.copySign(value * value, value);

      return value;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return (Command) m_autoChooser.getSelected();
  }
}
