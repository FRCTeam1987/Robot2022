// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CollectBalls;
import frc.robot.commands.PowercycleLimelight;
import frc.robot.commands.ResetLimelightPipeline;
import frc.robot.commands.auto.BlueFiveBallAuto;
import frc.robot.commands.auto.BlueThreeBallAuto;
import frc.robot.commands.auto.FiveBallAuto;
import frc.robot.commands.auto.OneBallAndD;
import frc.robot.commands.auto.OneBallAndSteal;
import frc.robot.commands.auto.TaxiAuto;
import frc.robot.commands.auto.ThreeBallAuto;
import frc.robot.commands.auto.ThreeBallSteal;
import frc.robot.commands.auto.TwoBallAndDAuto;
import frc.robot.commands.auto.TwoBallAndDHubAuto;
import frc.robot.commands.auto.TwoBallAndOneDAuto;
import frc.robot.commands.auto.TwoBallAndOneHubAuto;
import frc.robot.commands.auto.TwoBallSteal;
import frc.robot.commands.climber.BrakeClimber;
import frc.robot.commands.climber.ClimberArmExtend;
import frc.robot.commands.climber.ClimberExtend;
import frc.robot.commands.climber.ClimberPivotDown;
import frc.robot.commands.climber.ClimberPivotUp;
import frc.robot.commands.climber.ClimberPullIn;
import frc.robot.commands.climber.ClimberPullUp;
import frc.robot.commands.climber.ClimberShift;
import frc.robot.commands.climber.ClimberToHome;
import frc.robot.commands.climber.CoastClimber;
import frc.robot.commands.climber.ZeroClimber;
import frc.robot.commands.collector.StowCollector;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.RotationToHub;
import frc.robot.commands.shooter.EjectOneBallBottom;
import frc.robot.commands.shooter.EjectOneBallTop;
import frc.robot.commands.shooter.LowerHood;
import frc.robot.commands.shooter.RaiseHood;
import frc.robot.commands.shooter.SetShooterRpm;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.storage.FeedShooter;
import frc.robot.commands.storage.SetBallCount;
import frc.robot.commands.storage.StopStorage;
import frc.robot.commands.storage.StorageDefault;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final PowerDistribution m_powerDistribution = new PowerDistribution();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final CollectorSubsystem m_collector = new CollectorSubsystem();
  private final StorageSubsystem m_storage = new StorageSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final LimeLight m_limelight = new LimeLight();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  private final XboxController controller = new XboxController(0);
  private final XboxController coController = new XboxController(1);
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  private boolean m_ShouldEjectOpponentBall = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    // drivetrain.register();
    // collector.register();

    m_drivetrain.setDefaultCommand(new DriveCommand(
            m_drivetrain,
            () -> -modifyAxis(controller.getLeftY()), // Axes are flipped here on purpose
            () -> -modifyAxis(controller.getLeftX()),
            () -> -modifyAxis(controller.getRightX()),
            // () -> !controller.getLeftStickButton()  // DRIVER MAP TO PREFERRENCE
            () -> controller.getLeftTriggerAxis() < 0.9  // DRIVER MAP TO PREFERRENCE
    ));

    m_storage.setDefaultCommand(new StorageDefault(m_storage));;

    configureButtonBindings();
    configureShuffleboard();
    m_compressor.enableAnalog(100, 120);

    m_powerDistribution.setSwitchableChannel(true);
  }

  public void rememberStartingPosition() {
    m_drivetrain.rememberStartingPosition();
  }

  public void reZeroFromStartingPositon() {
    m_drivetrain.reZeroFromStartingPositon();
  }

  public void psiDashboardUpdate() {
    SmartDashboard.putNumber("AIR PRESSURE", m_compressor.getPressure());
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
      .whenPressed(new CollectBalls(controller, m_collector, m_storage, 2));
    new Button(controller::getRightBumperReleased)
      .whenReleased(new StowCollector(m_collector)
        .andThen(new StopStorage(m_storage)));
    new Button(controller::getLeftBumper)
      .whileHeld(  
        new ConditionalCommand(
          shootCommandHelper(),
        // new SetRumble(controller, RumbleValue.On).withTimeout(0.5).andThen(new SetRumble(controller, RumbleValue.Off)),
        new ParallelCommandGroup(
          new InstantCommand(() -> controller.setRumble(RumbleType.kLeftRumble, 1)), 
          new InstantCommand(() -> controller.setRumble(RumbleType.kRightRumble, 1))),
        () -> m_limelight.canSeeTarget())
      ).whenReleased(
        new ParallelCommandGroup(
          new InstantCommand(() -> controller.setRumble(RumbleType.kLeftRumble, 0)), 
          new InstantCommand(() -> controller.setRumble(RumbleType.kRightRumble, 0))
        ));

    new POVButton(controller, 180)
      .whenPressed(new InstantCommand(() -> {
        m_climberSubsystem.climberRightRetract();
        m_climberSubsystem.climberLeftRetract();
      }, m_climberSubsystem))
      .whenReleased(new InstantCommand(() -> m_climberSubsystem.climberStop(), m_climberSubsystem));

    new POVButton(controller, 0)
      .whenPressed(new InstantCommand(() -> {
        m_climberSubsystem.climberRightExtend();
        m_climberSubsystem.climberLeftExtend();
      }, m_climberSubsystem))
      .whenReleased(new InstantCommand(() -> m_climberSubsystem.climberStop(), m_climberSubsystem));

    new Button(coController::getLeftBumper)
      .whenPressed(new EjectOneBallTop(m_storage, m_shooter));
    new Button(coController::getRightBumper)
      .whenPressed(new EjectOneBallBottom(m_storage, m_collector));
    new POVButton(coController, 0)
      .whenPressed(new SetBallCount(m_storage, 2));
    new POVButton(coController, 90)
      .whenPressed(new SetBallCount(m_storage, 1));
    new POVButton(coController, 180)
      .whenPressed(new SetBallCount(m_storage, 0));

    new Button(controller::getYButton)
      .whenPressed(
        // new SequentialCommandGroup(
        //   new ClimberPivotUp(m_climberSubsystem),
          new ClimberExtend(m_climberSubsystem)
        // )
      );
    new Button(controller::getAButton)
      .whenPressed(new ClimberPullUp(m_climberSubsystem));
    new Button(coController::getYButton)
      .whenPressed(new ClimberShift(m_climberSubsystem, m_drivetrain));
    new Button(coController::getAButton)
      .whenPressed(new ClimberPullIn(m_climberSubsystem));

    new Button(controller::getBButton)
    .whileHeld(new Shoot(m_shooter, m_storage, m_drivetrain, m_limelight));
  
    new Button(coController::getBackButton)
    .whenPressed(new InstantCommand(() -> m_shooter.decrementOffsetRPM()));

    new Button(coController::getStartButton)
    .whenPressed(new InstantCommand(() -> m_shooter.incrementOffsetRPM()));
  };

  private void configureShuffleboard() {
    m_autoChooser.addOption("5 Ball Auto", new FiveBallAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    m_autoChooser.addOption("Blue 5 Ball", new BlueFiveBallAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    m_autoChooser.addOption("Taxi Auto", new TaxiAuto(controller, m_drivetrain, this));
    m_autoChooser.addOption("2 Ball & D Auto", new TwoBallAndDAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    m_autoChooser.addOption("2 Ball & Hub D Auto", new TwoBallAndDHubAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    m_autoChooser.addOption("2 Ball & 1 D Auto", new TwoBallAndOneDAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    m_autoChooser.addOption("2 Ball & 1 Hub Auto", new TwoBallAndOneHubAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    m_autoChooser.addOption("1 Ball & D Auto", new OneBallAndD(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    m_autoChooser.addOption("2 Ball & D Steal", new TwoBallSteal(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    m_autoChooser.addOption("1 Ball & Steal", new OneBallAndSteal(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    m_autoChooser.addOption("3 Ball & Steal", new ThreeBallSteal(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));

    m_autoChooser.addOption("3 Ball Auto", new ThreeBallAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    m_autoChooser.addOption("Blue 3 Ball", new BlueThreeBallAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    
    m_autoChooser.addOption("Test",  m_drivetrain.followPathCommand(false, "Test"));
    // m_autoChooser.addOption("Swerve Char - Forwards", new SwerveCharacterizationFF(m_drivetrain, true, false));
    
    m_autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    // m_autoChooser.addOption("Swerve Char - Reverse", new SwerveCharacterizationFF(m_drivetrain, false, false));
    // m_autoChooser.addOption("Swerve Char - Rotate", new SwerveCharacterizationFF(m_drivetrain, true, true));
    SmartDashboard.putData(m_autoChooser);

    // SmartDashboard.putData("Collector-Deploy", new DeployCollector(m_collector));
    // SmartDashboard.putData("Collector-Stow", new StowCollector(m_collector));
    SmartDashboard.putData("Feed Shooter", new FeedShooter(m_storage));
    SmartDashboard.putData("Shooter-Spin", new SetShooterRpm(m_shooter, () -> SmartDashboard.getNumber("RPM-Set", 0.0) ));
    SmartDashboard.putData("Shooter-Stop", new SetShooterRpm(m_shooter, () -> 0.0));
    // SmartDashboard.putData("Store-In", new RunStorageIn(m_storage));
    // SmartDashboard.putData("Store-Out", new RunStorageOut(m_storage));
    // SmartDashboard.putData("Store-Stop", new StopStorage(m_storage));
    // SmartDashboard.putData("LL-Standard", new ChangeLimeLightStream(StreamType.standard));
    // SmartDashboard.putData("LL-PipMain", new ChangeLimeLightStream(StreamType.pipMain));
    // SmartDashboard.putData("LL-PipSecondary", new ChangeLimeLightStream(StreamType.pipSecondary));
    SmartDashboard.putData("shoot", new Shoot(m_shooter, m_storage, m_drivetrain, m_limelight));
    // SmartDashboard.putData("Reset Ball Count", new ZeroBallCount(m_storage));
    // SmartDashboard.putData("Rotate-45", new RotateToAngle(m_drivetrain, () -> Rotation2d.fromDegrees(45).getRadians()));
    // SmartDashboard.putData("Rotate-LL", new RotateToLimelightAngle(m_drivetrain, m_limelight));
    // SmartDashboard.putData("Climber Extend", new ClimberExtend(m_climberSubsystem));
    // SmartDashboard.putData("Climber Pull-Up", new ClimberPullUp(m_climberSubsystem));
    // SmartDashboard.putData("Climber Shift", new ClimberShift(m_climberSubsystem));
    // SmartDashboard.putData("Climber Pull-In", new ClimberPullIn(m_climberSubsystem));
    // SmartDashboard.putData("Pivot Down", new ClimberPivotDown(m_climberSubsystem));
    // SmartDashboard.putData("Pivot Up", new ClimberPivotUp(m_climberSubsystem));
    SmartDashboard.putData("Climber to Home", new ClimberToHome(m_climberSubsystem));
    SmartDashboard.putData("Coast Climber", new CoastClimber(m_climberSubsystem));
    SmartDashboard.putData("Brake Climber", new BrakeClimber(m_climberSubsystem));
    SmartDashboard.putData("Zero Climber", new ZeroClimber(m_climberSubsystem));
    
    SmartDashboard.putData("Extend Climber", new ClimberArmExtend(m_climberSubsystem, m_drivetrain, 24));
    
    SmartDashboard.putData("Powercycle Limelight", new PowercycleLimelight(m_powerDistribution));
    SmartDashboard.putData("Reset Limelight Pipeline", new ResetLimelightPipeline(m_limelight));
    SmartDashboard.putData("Pivot Up", new ClimberPivotUp(m_climberSubsystem));
    SmartDashboard.putData("Pivot Down", new ClimberPivotDown(m_climberSubsystem));
    SmartDashboard.putData("Increment Offset", new InstantCommand(() -> m_shooter.incrementOffsetRPM()));
    SmartDashboard.putData("Decrement Offset", new InstantCommand(() -> m_shooter.decrementOffsetRPM()));
    SmartDashboard.putData("Raise Shoot Hood", new RaiseHood(m_shooter));
    SmartDashboard.putData("Lower Shoot Hood", new LowerHood(m_shooter));
    SmartDashboard.putData("Rotate to Hub", new RotationToHub(m_drivetrain)); //untested
    SmartDashboard.putBoolean("Should Use Color Ejection", m_ShouldEjectOpponentBall); //untested
    SmartDashboard.putData("Eject Op. Ball", ejectOpponentBall());

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

  public Command shootCommandHelper() {
    return new ConditionalCommand(
      new Shoot(
        m_shooter,
        m_storage,
        m_drivetrain,
        m_limelight,
        () -> m_shooter.getRPMFromLimelight() * Constants.Shooter.SHOOTER_REDUCTION,//m_limelight.getYAxis() < -5 ? 3075 : 2500,
        () -> m_limelight.getYAxis() < 11
      ),
      new ConditionalCommand(
        new InstantCommand(() -> controller.setRumble(RumbleType.kLeftRumble, 1))
          .andThen(new WaitCommand(0.25))
          .andThen(new InstantCommand(() -> controller.setRumble(RumbleType.kLeftRumble, 0))), 
          new InstantCommand(), 
          () -> DriverStation.isAutonomousEnabled() == false), 
      () -> m_limelight.getYAxis() > -14 && m_limelight.getYAxis() < 16.3);

    // if (m_limelight.getYAxis() > -12 && m_limelight.getYAxis() < 4) {
      // return new Shoot(
      //   m_shooter,
      //   m_storage,
      //   m_drivetrain,
      //   m_limelight,
      //   () -> m_shooter.getRPMFromLimelight(),//m_limelight.getYAxis() < -5 ? 3075 : 2500,
      //   () -> m_limelight.getYAxis() < -7.5 ? 50 : 65
      // );
    // } else if (DriverStation.isAutonomousEnabled() == false) {
    //     return new InstantCommand(() -> controller.setRumble(RumbleType.kLeftRumble, 1))
    //       .andThen(new WaitCommand(0.25))
    //       .andThen(new InstantCommand(() -> controller.setRumble(RumbleType.kLeftRumble, 0)));
    // } else {
    //   return new InstantCommand();
    // }
    
  }
  public ConditionalCommand ejectOpponentBall() {  //FIXME This code  deals with ball ejeection based on color
    return new ConditionalCommand(
      new SequentialCommandGroup(
        new ConditionalCommand(
          new EjectOneBallBottom(m_storage, m_collector),
          new InstantCommand(),
          () -> !m_storage.isBottomOurs()
        ),
        new ConditionalCommand(
          new EjectOneBallTop(m_storage, m_shooter),
          new InstantCommand(),
        () -> !m_storage.isTopOurs() && m_storage.getBallCount() != 2
      )
    ), 
    new InstantCommand(), 
    () -> (SmartDashboard.getBoolean("Should Use Color Ejection", false)
    ));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   * 
   */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return (Command) m_autoChooser.getSelected();
  }
}
