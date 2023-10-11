// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtensionConstants;

public class ExtensionSubsystem extends SubsystemBase {
  private final CANSparkMax ArmExtensionMotor;
  private final DigitalInput rearMagneticLimit;
  private final DigitalInput forwardMagneticLimit;
  /** Creates a new ExtensionSubsystem. */
  public ExtensionSubsystem() {
    //Motor
    ArmExtensionMotor = initMotor(ExtensionConstants.kArmExtensionMotorID);
    //Sensors
    rearMagneticLimit = new DigitalInput(ExtensionConstants.kRearMagneticLimitID);
    forwardMagneticLimit = new DigitalInput(ExtensionConstants.kForwardMagneticLimitID);
  }
  private CANSparkMax initMotor(int canID){
    CANSparkMax newMotor = new CANSparkMax(canID, MotorType.kBrushless);
    newMotor.restoreFactoryDefaults();
    newMotor.setIdleMode(IdleMode.kBrake);
    newMotor.burnFlash();
    return newMotor;
  }
  public void setArmExtensionMotor(double speed){
    ArmExtensionMotor.set(speed);
  }
  public void safeSetArmExtensionMotor(double speed){
    double safeSpeed = speed;
    //If we are at the the rear limit and trying to run backwards
    if(getRearLimit() && speed < 0){
      safeSpeed = 0;
    }
    if(getForwardLimit() && speed > 0){
      safeSpeed = 0;
    }
    ArmExtensionMotor.set(safeSpeed);
  }
  public boolean getRearLimit(){
    return !rearMagneticLimit.get();
  }
  public boolean getForwardLimit(){
    return !forwardMagneticLimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Forward Mag Limit", getForwardLimit());
    SmartDashboard.putBoolean("Rear Mag Limit", getRearLimit());
  }
}
