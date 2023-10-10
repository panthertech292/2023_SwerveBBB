// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  //Motors
  private final CANSparkMax ArmRotateMotor;
  private final CANSparkMax ArmExtensionMotor;
  //Sensors
  private final DutyCycleEncoder ArmRotateEncoder;
  private final DigitalInput rearMagneticLimit;
  private final DigitalInput forwardMagneticLimit;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    //Motors
    ArmRotateMotor = initMotor(ArmConstants.kArmRotateMotorID);
    ArmExtensionMotor = initMotor(ArmConstants.kArmExtensionMotorID);
    //Sensors
    ArmRotateEncoder = new DutyCycleEncoder(ArmConstants.kArmRotateEncoderID);
    rearMagneticLimit = new DigitalInput(ArmConstants.kRearMagneticLimitID);
    forwardMagneticLimit = new DigitalInput(ArmConstants.kForwardMagneticLimitID);
  }

  private CANSparkMax initMotor(int canID){
    CANSparkMax newMotor = new CANSparkMax(canID, MotorType.kBrushless);
    newMotor.restoreFactoryDefaults();
    newMotor.setIdleMode(IdleMode.kBrake);
    newMotor.burnFlash();
    return newMotor;
  }

  public void setArmRotateMotor(double speed){
    ArmRotateMotor.set(speed);
  }
  /* 
  TODO: Figure out if this is needed
  We might just be able to use ArmRotateMotor.enableSoftLimit(direction, enable)

  public void safeSetArmRotateMotor(double speed){
    double safeSpeed = speed;
    //If we are on or pass the min limit and we are trying to run the motor in the negative direction
    if ((getArmRotatePosition() < ArmConstants.kMinArmTheta) && speed < 0){
      safeSpeed = 0; //Halt the motor as further travel in this direction is harmful
    }
    //If we are on or pass the max limit and we are trying to run the motor in the positive direction
    if ((getArmRotatePosition() > ArmConstants.kMaxArmTheta) && speed > 0){
      safeSpeed = 0; //Halt the motor as further travel in this direction is harmful
    }
    ArmRotateMotor.set(safeSpeed);
  }*/
  public void setArmExtensionMotor(double speed){
    ArmExtensionMotor.set(speed);
  }
  public double getArmRotatePosition(){
    return ArmRotateEncoder.getAbsolutePosition();
  }
  //TODO: Check if these need inverted. Probably will need it.
  public boolean getRearLimit(){
    return rearMagneticLimit.get();
  }
  public boolean getForwardLimit(){
    return forwardMagneticLimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
