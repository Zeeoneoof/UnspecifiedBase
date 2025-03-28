// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  private double gear = 0.2;
  private int speedModifier = 1;
  private CommandXboxController modifier = RobotContainer.m_driverController;
  private SlewRateLimiter limiter = new SlewRateLimiter(1);
  private SlewRateLimiter limiter2 = new SlewRateLimiter(1);

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()).unaryMinus(),
      // Swap init pose with something later
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, Pose2d.kZero);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // Configure PathPlanner for auto trajectory planning
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );



  } 
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", m_gyro.getYaw());
    /*
     * Odometry
     */
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()).unaryMinus(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
      
    updateOdometryWithVision();
    /*
     * Gearing 
     */
    // Using a switch case to assign gears to multipliers, then report gearing status to dashboard
    if (modifier.rightBumper().getAsBoolean()){
      speedModifier = (int) limiter.calculate(Math.max(1, Math.min(4, speedModifier+1)));
    }
  
    // Gear Down when Left Bumper true
    if (modifier.leftBumper().getAsBoolean()){
      speedModifier = (int) limiter2.calculate(Math.max(1, Math.min(4, speedModifier-1)));
    }

    switch (speedModifier) {
      case 1:
        gear = 0.20;
        SmartDashboard.putString("Gearbox Status","Gearing should be at slowest speed");
        break;
      case 2:
        gear = 0.33;
        SmartDashboard.putString("Gearbox Status","Gearing should be at stable medium");
        break;
      case 3:
        gear = 0.70;
        SmartDashboard.putString("Gearbox Status","Gearing should be at max speed");
        break;
      case 4:
        gear = 1.00;
        SmartDashboard.putString("Gearbox Status", "Robot is in overdrive");
      default:
        break; 
    }
    SmartDashboard.putNumber("Gear", gear);
    SmartDashboard.putNumber("Speed Modifier", speedModifier);
    Logger.recordOutput("EstimatedPose", getPose());
  }

  /**
   * Updates odometry via vision
   */
  public void updateOdometryWithVision(){
    boolean useMegaTag2 = false; //set to false to use MegaTag1
    boolean doRejectUpdateA = false;
    boolean doRejectUpdateB = false;
    try{
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-a");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7 || mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdateA = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdateA = true;
      }

      if(mt2.tagCount == 1 && mt2.rawFiducials.length == 1)
      {
        if(mt2.rawFiducials[0].ambiguity > .7 || mt2.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdateB = true;
        }
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdateB = true;
      }

      if(!doRejectUpdateA)
      {
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_odometry.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
      if(!doRejectUpdateB)
      {
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_odometry.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight-a", Rotation2d.fromDegrees(m_gyro.getAngle()).unaryMinus().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight", Rotation2d.fromDegrees(m_gyro.getAngle()).unaryMinus().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2a = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-a");
      LimelightHelpers.PoseEstimate mt2b = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdateA = true;
        doRejectUpdateB = true;
      }
      if(mt2a.tagCount == 0)
      {
        doRejectUpdateA = true;
      } 
      if (mt2b.tagCount == 0){
        doRejectUpdateB  = true;
      }
      if(!doRejectUpdateA)
      {
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_odometry.addVisionMeasurement(
            mt2a.pose,
            mt2a.timestampSeconds);
      }
      if(!doRejectUpdateB){
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_odometry.addVisionMeasurement(
            mt2b.pose,
            mt2b.timestampSeconds);
      }

    }
  }
  catch (Exception e){
    System.err.println("Limelight not initialized or not connected"+e);
  }
  }


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @AutoLogOutput
  public Pose2d getPose() {
    
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()).unaryMinus(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = gear*xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = gear*ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = gear*rot * DriveConstants.kMaxAngularSpeed;



    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle()).unaryMinus()), 0.02)
            : ChassisSpeeds.discretize(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered), 0.02));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveRelative(ChassisSpeeds speeds) {
    // Convert the commanded speeds into the correct units for the drivetrain
    boolean fieldRelative = false;
    double xSpeedDelivered = speeds.vxMetersPerSecond;
    double ySpeedDelivered = speeds.vyMetersPerSecond;
    double rotDelivered = speeds.omegaRadiansPerSecond;



    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle()).unaryMinus())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }


  /** Return Robot-Relative Speeds */
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                                                           m_frontRight.getState(),
                                                           m_rearLeft.getState(),
                                                           m_rearRight.getState());
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }


  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }



  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  @AutoLogOutput
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).unaryMinus().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
