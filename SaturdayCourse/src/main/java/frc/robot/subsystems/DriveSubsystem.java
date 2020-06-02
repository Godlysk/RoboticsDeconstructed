/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  
  
  private final WPI_TalonSRX FR;
  private final WPI_TalonSRX BR;
  private final SpeedControllerGroup rightSide;

  private final WPI_TalonSRX FL;
  private final WPI_TalonSRX BL;  
  private final SpeedControllerGroup leftSide;

  private final DifferentialDrive driveTrain;
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    FR = new WPI_TalonSRX(Constants.FR_port);
    BR = new WPI_TalonSRX(Constants.BR_port);
    rightSide = new SpeedControllerGroup(FR, BR);
    
    FL = new WPI_TalonSRX(Constants.FL_port);
    BL = new WPI_TalonSRX(Constants.BL_port); 
    leftSide = new SpeedControllerGroup(FL, BL);
    
    driveTrain = new DifferentialDrive(leftSide, rightSide);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(final double l, final double r) {
    FR.setInverted(true);
    BR.setInverted(true);

    FR.set(r);
    BR.set(r);
    FL.set(l);
    BL.set(l);
  }

  public void arcadeInbuilt(final double y, final double z) {
    FR.setInverted(false);
    BR.setInverted(false);

    driveTrain.arcadeDrive(y * Constants.maxSpeed, z * Constants.maxSpeed);
  }

  double integralEncoder = 0;
  double previousErrorEncoder = 0;

  public void drivePIDEncoder(double yaxis) {

    // Calculate Error
    double leftRate = RobotContainer.enc_L.getRate();
    double rightRate = RobotContainer.enc_R.getRate();
    double error = leftRate - rightRate;

    // Calculate Correction
    integralEncoder += error;
    if (yaxis < Constants.integralResetBound) integralEncoder = 0;

    double derivative = error - previousErrorEncoder;
    previousErrorEncoder = error;

    double correction = ((error * Constants.kPEncoder) + (integralEncoder * Constants.kIEncoder) + (derivative * Constants.kDEncoder));

    // Apply the Correction to Motor Voltages
    double leftSpeed = yaxis - correction;
    double rightSpeed = yaxis + correction;

    drive(leftSpeed * Constants.maxSpeed, rightSpeed * Constants.maxSpeed);

  }


  double integralNavX = 0;
  double previousErrorNavX = 0;

  public void drivePIDNavX(double yaxis) {

    // Calculate Error
    double navxYawRate = RobotContainer.navx.getRate();
    double error = navxYawRate;

    // Calculate Correction
    integralNavX += error;
    if (yaxis < Constants.integralResetBound) integralNavX = 0;

    double derivative = error - previousErrorNavX;
    previousErrorNavX = error;

    double correction = ((error * Constants.kPNavX) + (integralEncoder * Constants.kINavX) + (derivative * Constants.kDNavX));

    // Apply Correction to Motor Voltages
    double leftSpeed = yaxis - correction;
    double rightSpeed = yaxis + correction;

    drive(leftSpeed * Constants.maxSpeed, rightSpeed * Constants.maxSpeed);

  }


  // https://www.kauailabs.com/dist/frc/2020/navx_frc.json


}
