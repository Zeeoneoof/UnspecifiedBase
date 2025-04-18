// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.commands.drive.AlignOdoCoral;
import frc.robot.commands.drive.DriveToIntake;
import frc.robot.commands.intake.IntakeWheelAuto;
import frc.robot.commands.intake.OuttakeWheelAuto;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.derive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
  private final ClawSubsystem m_robotClaw = new ClawSubsystem();
  private final PivotSubsystem m_robotPivot = new PivotSubsystem();

  // Auto Builder
  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  public static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public CommandXboxController m_subsystemController = new CommandXboxController(OIConstants.kSubsystemDriverControllerPort);
  public final DigitalInput climberLimitSwitch = new DigitalInput(0);
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  IntakeWheelAuto spinIntake = new IntakeWheelAuto(m_robotClaw);
  OuttakeWheelAuto outtakeCoral = new OuttakeWheelAuto(m_robotClaw);
  private final Command cancelPivotCommand = Commands.runOnce(()->m_robotPivot.getCurrentCommand().cancel());
  private final Command cancelElevatorCommand = Commands.runOnce(()->m_robotElevator.getCurrentCommand().cancel());
  private final Command cancelDrivetrainCommand = Commands.runOnce(()->m_robotDrive.getCurrentCommand().cancel());
  public RobotContainer() {
    NamedCommands.registerCommand("StowCommand", Commands.race(m_robotElevator.setElevator(4,2).andThen(m_robotPivot.stowCommand()), Commands.waitSeconds(4)));
    NamedCommands.registerCommand("ChangeToL4", Commands.race(m_robotElevator.setElevator(3,1).andThen(m_robotPivot.setPivot(3,1)), Commands.waitSeconds(4)));
    NamedCommands.registerCommand("IntakeCommand", Commands.race(m_robotElevator.setElevator(4,1).andThen(m_robotPivot.setPivot(4,1)), Commands.waitSeconds(4)));
    NamedCommands.registerCommand("IntakeWheelCommand", spinIntake);
    NamedCommands.registerCommand("IntakeOuttakeCommand", Commands.race(new OuttakeWheelAuto(m_robotClaw), Commands.waitSeconds(4)));
    NamedCommands.registerCommand("ChangeToL2", Commands.race(m_robotElevator.setElevator(1,1).andThen(m_robotPivot.setPivot(1,1)), Commands.waitSeconds(2)));
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
     
    // Add named commands
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    LEDConstants.m_led.setLength(LEDConstants.m_ledBuffer.getLength());
    LEDConstants.m_led.setData(LEDConstants.m_ledBuffer);
    LEDConstants.m_led.start();
    
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
     * Driver controls
     */

    // Climber
    m_driverController.back().whileTrue(Commands.runEnd(()->m_robotElevator.spinClimbMotor(-0.5),()->m_robotElevator.stopClimbMotor(),m_robotElevator));
    m_driverController.start().whileTrue(Commands.runEnd(()->m_robotElevator.spinClimbMotor(0.5),()->m_robotElevator.stopClimbMotor(),m_robotElevator));
    
    // Gyro reset
    m_driverController.a().onTrue(Commands.runOnce(()->m_robotDrive.zeroHeading()));

    m_driverController.x().onTrue(new AlignOdoCoral(m_robotDrive,true));
    m_driverController.leftTrigger().onTrue(cancelDrivetrainCommand);
    m_driverController.b().onTrue(new AlignOdoCoral(m_robotDrive, false));
    m_driverController.y().onTrue(new DriveToIntake(m_robotDrive, false));

    /*
     *  Subsystem Driver controls
     */
    
     // Limit switch for elevator
    new Trigger(climberLimitSwitch::get).onTrue(m_robotElevator.resetEncoder().andThen(Commands.waitSeconds(2)));

    // Manual voltage controls for elevator
    // m_subsystemController.y().whileTrue(Commands.runEnd(()->m_robotElevator.runWithVoltage(0.1), ()->m_robotElevator.stopMotor(), m_robotElevator));
    // m_subsystemController.a().whileTrue(Commands.runEnd(()->m_robotElevator.runWithVoltage(-0.1), ()->m_robotElevator.stopMotor(), m_robotElevator));

    // Button controls (LMHSI positions)
    
    /*
     * Coral (LB + Button)
     */
    

    m_subsystemController.x().and(m_subsystemController.leftBumper()).onTrue(m_robotElevator.setElevator(0,1).andThen(m_robotPivot.setPivot(0,1))); // Low
    m_subsystemController.y().and(m_subsystemController.leftBumper()).onTrue(m_robotElevator.setElevator(1,1).andThen(m_robotPivot.setPivot(1,1))); // Mid
    m_subsystemController.b().and(m_subsystemController.leftBumper()).onTrue(m_robotElevator.setElevator(2,1).andThen(m_robotPivot.setPivot(2,1))); // High
    m_subsystemController.a().and(m_subsystemController.leftBumper()).onTrue(m_robotElevator.setElevator(3,1).andThen(m_robotPivot.setPivot(3,1))); // Special
    m_subsystemController.start().and(m_subsystemController.leftBumper()).onTrue(m_robotElevator.setElevator(4,1).andThen(m_robotPivot.setPivot(4,1))); // Intake
    /*
     * Algae (RB + Button)
     */

 
   


    m_subsystemController.x().and(m_subsystemController.rightBumper()).onTrue(m_robotElevator.setElevator(0,2).andThen(m_robotPivot.setPivot(0,2))); // Low
    m_subsystemController.y().and(m_subsystemController.rightBumper()).onTrue(m_robotElevator.setElevator(1,2).andThen(m_robotPivot.setPivot(1,2))); // Mid
    m_subsystemController.b().and(m_subsystemController.rightBumper()).onTrue(m_robotElevator.setElevator(2,2).andThen(m_robotPivot.setPivot(2,2))); // High
    m_subsystemController.a().and(m_subsystemController.rightBumper()).onTrue(m_robotElevator.setElevator(3,2).andThen(m_robotPivot.setPivot(3,2))); // Special
    m_subsystemController.start().and(m_subsystemController.rightBumper()).onTrue(m_robotElevator.setElevator(4,2).andThen(m_robotPivot.setPivot(4,2))); // Intake

    // TODO
    // m_subsystemController.rightBumper().onTrue(Commands.runOnce(()->m_robotElevator.setMode(2),m_robotElevator).alongWith(Commands.runOnce(()->m_robotPivot.setMode(2), m_robotPivot)));
    // m_subsystemController.leftBumper().onTrue(Commands.runOnce(()->m_robotElevator.setMode(1),m_robotElevator).alongWith(Commands.runOnce(()->m_robotPivot.setMode(1), m_robotPivot)));
    // Claw intake
    m_subsystemController.leftTrigger().toggleOnTrue(Commands.runEnd(()->m_robotClaw.runWheelWithVoltage(-0.8), ()->m_robotClaw.stopWheelMotor(), m_robotClaw));
    m_subsystemController.rightTrigger().whileTrue(Commands.runEnd(()->m_robotClaw.runWheelWithVoltage(0.23), ()->m_robotClaw.stopWheelMotor(), m_robotClaw));
    
    

    /*
     * Manual Adjustments
     */

    // Elevator Manual Rotation
     m_subsystemController.povUp().whileTrue(Commands.runEnd(()->m_robotElevator.manualAdjustmentFunc(1.5),()->Commands.none(),m_robotElevator));
     m_subsystemController.povDown().whileTrue(Commands.runEnd(()->m_robotElevator.manualAdjustmentFunc(-1.5),()->Commands.none(),m_robotElevator));
    // Claw Manual Rotation
    m_subsystemController.povLeft().whileTrue(Commands.runEnd(()->m_robotPivot.manualAdjustmentFunc(-0.025/2),()->Commands.none(),m_robotPivot));
    m_subsystemController.povRight().whileTrue(Commands.runEnd(()->m_robotPivot.manualAdjustmentFunc(0.025/2),()->Commands.none(),m_robotPivot));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
  /*  // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0.2)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
 // }
 */
    
   return autoChooser.getSelected();
  }
}
