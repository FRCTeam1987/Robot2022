// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
import frc.robot.commands.climber.BrakeTelescope;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.ClimbStep1;
import frc.robot.commands.climber.ClimbStep2;
// import frc.robot.commands.climber.ClimberPivotDown;
// import frc.robot.commands.climber.ClimberPivotUp;
// import frc.robot.commands.climber.ClimberPullIn;
import frc.robot.commands.climber.ClimbStep3;
import frc.robot.commands.climber.ClimbStep4;
import frc.robot.commands.climber.ClimberToHome;
import frc.robot.commands.climber.CoastClimber;
import frc.robot.commands.climber.DisengageFrictionBrakeClimber;
import frc.robot.commands.climber.DisengageFrictionBrakeTelescope;
import frc.robot.commands.climber.EngageFrictionBrakeClimber;
import frc.robot.commands.climber.TelescopeAutoHome;
import frc.robot.commands.climber.TelescopeGoToClosedLoop;
import frc.robot.commands.climber.ZeroClimber;
import frc.robot.commands.collector.DeployCollector;
import frc.robot.commands.collector.StowCollector;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.RotationToHub;
import frc.robot.commands.drivetrain.ZeroRoll;
import frc.robot.commands.shooter.EjectOneBallBottom;
import frc.robot.commands.shooter.EjectOneBallTop;
import frc.robot.commands.shooter.LowerHood;
import frc.robot.commands.shooter.RaiseHood;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.storage.SetBallCount;
import frc.robot.commands.storage.StopStorage;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
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
  private Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
  private PowerDistribution m_powerDistribution = new PowerDistribution();
  private DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private CollectorSubsystem m_collector = new CollectorSubsystem();
  private StorageSubsystem m_storage = new StorageSubsystem();
  private ShooterSubsystem m_shooter = new ShooterSubsystem();
  private LimeLight m_limelight = new LimeLight();
  private TelescopeSubsystem m_telescopeFront = new TelescopeSubsystem(Constants.Climber.Front_CLIMBER_MOTOR, false, "Front", Constants.Climber.FRONT_TELESCOPE_SOLENOID);
  private TelescopeSubsystem m_telescopeBack = new TelescopeSubsystem(Constants.Climber.Back_CLIMBER_MOTOR, true, "Back", Constants.Climber.BACK_TELESCOPE_SOLENOID);

  private XboxController controller = new XboxController(0);
  private XboxController coController = new XboxController(1);
  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

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

    // m_storage.setDefaultCommand(new StorageDefault(m_storage));;

    configureButtonBindings();
    configureShuffleboard();
    m_compressor.enableAnalog(100, 120);

    m_powerDistribution.setSwitchableChannel(true);
    // LiveWindow.disableAllTelemetry();
  }

  public void rememberStartingPosition() {
    m_drivetrain.rememberStartingPosition();
  }

  public void reZeroFromStartingPositon() {
    m_drivetrain.reZeroFromStartingPositon();
  }

  public void psiDashboardUpdate() {
    // SmartDashboard.putNumber("AIR PRESSURE", m_compressor.getPressure());
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

    // new POVButton(controller, 180)
    //   .whenPressed(new InstantCommand(() -> {
    //     m_climberFrontSubsystem.climberRetract(0.75);
    //     m_climberBackSubsystem.climberRetract(0.75);
    //   }, m_climberFrontSubsystem, m_climberFrontSubsystem))
    //   .whenReleased(new InstantCommand(() -> {
    //     m_climberFrontSubsystem.climberStop();
    //     m_climberBackSubsystem.climberStop();
    //   }, m_climberFrontSubsystem, m_climberBackSubsystem));

    // new POVButton(controller, 0)
    //   .whenPressed(new InstantCommand(() -> {
    //     m_climberFrontSubsystem.climberExtend(0.75);
    //     m_climberBackSubsystem.climberExtend(0.75);
    //   }, m_climberFrontSubsystem, m_climberBackSubsystem))
    //   .whenReleased(new InstantCommand(() -> {
    //     m_climberFrontSubsystem.climberStop();
    //     m_climberBackSubsystem.climberStop();
    //   }, m_climberFrontSubsystem, m_climberBackSubsystem));

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
        new ParallelCommandGroup(
          // new ClimberBackExtend(m_climberSubsystem),
          // new ClimberFrontExtend(m_climberSubsystem)
        )
        // )
      );
    // new Button(controller::getAButton)
    //   .whenPressed(new ClimberFrontRetract(m_climberSubsystem, 0.4));
    // new Button(coController::getYButton)
    //   .whenPressed(new ClimbStep3(m_climberFrontSubsystem, m_climberBackSubsystem, m_drivetrain));
    // new Button(coController::getAButton) //TODO REPLACE WITH NEW COMMAND
    //   .whenPressed(new ClimberPullIn(m_climberSubsystem));

    new Button(controller::getBButton)
    .whileHeld(new Shoot(m_shooter, m_storage, m_drivetrain, m_limelight));
  
    new Button(coController::getBackButton)
    .whenPressed(new InstantCommand(() -> m_shooter.decrementOffsetRPM()));

    new Button(coController::getStartButton)
    .whenPressed(new InstantCommand(() -> m_shooter.incrementOffsetRPM()));
  };

  private void configureShuffleboard() {
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
    ShuffleboardTab telescopesTab = Shuffleboard.getTab("Telescopes");
    ShuffleboardTab limeLightTab = Shuffleboard.getTab("LimeLight");

    // m_autoChooser.addOption("5 Ball Auto", new FiveBallAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    // m_autoChooser.addOption("Blue 5 Ball", new BlueFiveBallAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    // m_autoChooser.addOption("Taxi Auto", new TaxiAuto(controller, m_drivetrain, this));
    // m_autoChooser.addOption("2 Ball & D Auto", new TwoBallAndDAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    // m_autoChooser.addOption("2 Ball & Hub D Auto", new TwoBallAndDHubAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    // m_autoChooser.addOption("2 Ball & 1 D Auto", new TwoBallAndOneDAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    // m_autoChooser.addOption("2 Ball & 1 Hub Auto", new TwoBallAndOneHubAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    // m_autoChooser.addOption("1 Ball & D Steal", new OneBallAndD(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    // m_autoChooser.addOption("2 Ball & D Steal", new TwoBallSteal(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    // m_autoChooser.addOption("1 Ball & Steal", new OneBallAndSteal(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    // m_autoChooser.addOption("3 Ball & Steal", new ThreeBallSteal(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));

    // m_autoChooser.addOption("3 Ball Auto", new ThreeBallAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    // m_autoChooser.addOption("Blue 3 Ball", new BlueThreeBallAuto(controller, m_drivetrain, m_collector, m_storage, m_shooter, m_limelight, this));
    
    // m_autoChooser.addOption("Test",  m_drivetrain.followPathCommand(false, "Test"));
    // m_autoChooser.addOption("Swerve Char - Forwards", new SwerveCharacterizationFF(m_drivetrain, true, false));
    
    m_autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    // m_autoChooser.addOption("Swerve Char - Reverse", new SwerveCharacterizationFF(m_drivetrain, false, false));
    // m_autoChooser.addOption("Swerve Char - Rotate", new SwerveCharacterizationFF(m_drivetrain, true, true));
    SmartDashboard.putData(m_autoChooser);

        
    limeLightTab.add("Powercycle Limelight", new PowercycleLimelight(m_powerDistribution));
    limeLightTab.add("Reset Limelight Pipeline", new ResetLimelightPipeline(m_limelight));

    SmartDashboard.putData("Increment Offset", new InstantCommand(() -> m_shooter.incrementOffsetRPM()));
    SmartDashboard.putData("Decrement Offset", new InstantCommand(() -> m_shooter.decrementOffsetRPM()));
    SmartDashboard.putData("Collector-Deploy", new DeployCollector(m_collector));
    SmartDashboard.putData("Collector-Stow", new StowCollector(m_collector));
    // SmartDashboard.putData("Feed Shooter", new FeedShooter(m_storage));
    // SmartDashboard.putData("Shooter-Spin", new SetShooterRpm(m_shooter, () -> SmartDashboard.getNumber("RPM-Set", 0.0) ));
    // SmartDashboard.putData("Shooter-Stop", new SetShooterRpm(m_shooter, () -> 0.0));
    // SmartDashboard.putData("Store-In", new RunStorageIn(m_storage));
    // SmartDashboard.putData("Store-Out", new RunStorageOut(m_storage));
    // SmartDashboard.putData("Store-Stop", new StopStorage(m_storage));
    // SmartDashboard.putData("LL-Standard", new ChangeLimeLightStream(StreamType.standard));
    // SmartDashboard.putData("LL-PipMain", new ChangeLimeLightStream(StreamType.pipMain));
    // SmartDashboard.putData("LL-PipSecondary", new ChangeLimeLightStream(StreamType.pipSecondary));
    // SmartDashboard.putData("shoot", new Shoot(m_shooter, m_storage, m_drivetrain, m_limelight));
    // SmartDashboard.putData("Reset Ball Count", new ZeroBallCount(m_storage));
    // SmartDashboard.putData("Rotate-45", new RotateToAngle(m_drivetrain, () -> Rotation2d.fromDegrees(45).getRadians()));
    // SmartDashboard.putData("Rotate-LL", new RotateToLimelightAngle(m_drivetrain, m_limelight));
    // SmartDashboard.putData("Climber Extend", new ClimberExtend(m_climberSubsystem));
    // SmartDashboard.putData("Climber Pull-Up", new ClimberPullUp(m_climberSubsystem));
    // SmartDashboard.putData("Climber Shift", new ClimberShift(m_climberSubsystem));
    // SmartDashboard.putData("Climber Pull-In", new ClimberPullIn(m_climberSubsystem));
    // SmartDashboard.putData("Pivot Down", new ClimberPivotDown(m_climberSubsystem));
    // SmartDashboard.putData("Pivot Up", new ClimberPivotUp(m_climberSubsystem));
    SmartDashboard.putData("Coast Climber", new CoastClimber(m_telescopeFront, m_telescopeBack));
    SmartDashboard.putData("Brake Climber", new BrakeClimber(m_telescopeFront, m_telescopeBack));
    SmartDashboard.putData("Zero Climber", new ZeroClimber(m_telescopeFront, m_telescopeBack));
    
    // telescopesTab.add("Climber to Home", new ClimberToHome(m_telescopeFront, m_telescopeBack));
    telescopesTab.add("Coast Climber", new CoastClimber(m_telescopeFront, m_telescopeBack));
    telescopesTab.add("Brake Climber", new BrakeClimber(m_telescopeFront, m_telescopeBack));
    telescopesTab.add("Zero Climber", new ZeroClimber(m_telescopeFront, m_telescopeBack));
    
    // SmartDashboard.putData("Extend Climber", new ClimberArmExtend(m_climberFrontSubsystem, m_climberBackSubsystem, m_drivetrain, 24));
    
    SmartDashboard.putData("Powercycle Limelight", new PowercycleLimelight(m_powerDistribution));
    SmartDashboard.putData("Reset Limelight Pipeline", new ResetLimelightPipeline(m_limelight));
    // SmartDashboard.putData("Pivot Up", new ClimberPivotUp(m_climberSubsystem));
    // SmartDashboard.putData("Pivot Down", new ClimberPivotDown(m_climberSubsystem));
    // SmartDashboard.putData("Increment Offset", new InstantCommand(() -> m_shooter.incrementOffsetRPM()));
    // SmartDashboard.putData("Decrement Offset", new InstantCommand(() -> m_shooter.decrementOffsetRPM()));
    SmartDashboard.putData("Raise Shoot Hood", new RaiseHood(m_shooter));
    SmartDashboard.putData("Lower Shoot Hood", new LowerHood(m_shooter));
    SmartDashboard.putData("0 Count", new SetBallCount(m_storage, 0));
    SmartDashboard.putData("1 Count", new SetBallCount(m_storage, 1));
    SmartDashboard.putData("2 Count", new SetBallCount(m_storage, 2));
    SmartDashboard.putData("Zero Gyro", new InstantCommand(() -> { m_drivetrain.zeroGyroscope(); }));
    // SmartDashboard.putData("Rotate to Hub", new SequentialCommandGroup(
    //   new InstantCommand(() -> System.out.println("Pose X: " + Math.round(m_drivetrain.getPose().getX()) + " Pose Y: " + Math.round(m_drivetrain.getPose().getY()) + " Current Angle: " + m_drivetrain.getAdjustedHeading().getDegrees())),
    //   new RotationToHub(m_drivetrain)
    // )); //untested
    // SmartDashboard.putData("Rotate to Hub", ); //untested
    // SmartDashboard.putData("Climb", new Climb(m_telescopeFront, m_telescopeBack, m_drivetrain, controller));
    // telescopesTab.add("Climb", new Climb(m_telescopeFront, m_telescopeBack, m_drivetrain, controller));

    SmartDashboard.putData("Zero Roll", new ZeroRoll(m_drivetrain));
    telescopesTab.add("Zero Roll", new ZeroRoll(m_drivetrain));
    telescopesTab.add("Climber Auto Home", new ParallelCommandGroup(
      new TelescopeAutoHome(m_telescopeFront),
      new TelescopeAutoHome(m_telescopeBack)
    ));
    telescopesTab.add("Climb 1", new ClimbStep1(m_telescopeFront, m_compressor));
    telescopesTab.add("Climb 2", new ClimbStep2(m_telescopeFront, m_telescopeBack, m_drivetrain));
    telescopesTab.add("Climb 3", new ClimbStep3(m_telescopeFront, m_telescopeBack, m_drivetrain));
    telescopesTab.add("Climb 4", new ClimbStep4(m_telescopeFront, m_telescopeBack, m_drivetrain));
    telescopesTab.add("roll without offset", m_drivetrain.getRoll());
    telescopesTab.add("roll with offset", m_drivetrain.getRollWithOffset());
    telescopesTab.add("Climber Engage Friction Brake", new EngageFrictionBrakeClimber(m_telescopeFront, m_telescopeBack));
    telescopesTab.add("Climber Disengage Friction Brake", new DisengageFrictionBrakeClimber(m_telescopeFront, m_telescopeBack));

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
      () -> m_limelight.getYAxis() > -16 && m_limelight.getYAxis() < 16.3);

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
