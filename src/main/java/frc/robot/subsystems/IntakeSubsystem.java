// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  //Motors
  private final CANSparkMax IntakeMotorUp;
  private final CANSparkMax IntakeMotorDown;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    IntakeMotorUp = initMotor(IntakeConstants.kIntakeMotorUpID);
    IntakeMotorDown = initMotor(IntakeConstants.kIntakeMotorDownID);
  }
  private CANSparkMax initMotor(int canID){
    CANSparkMax newMotor = new CANSparkMax(canID, MotorType.kBrushless);
    newMotor.restoreFactoryDefaults();
    newMotor.setIdleMode(IdleMode.kBrake);
    newMotor.enableVoltageCompensation(12);
    newMotor.setSmartCurrentLimit(1, 20, 60); //Not sure about this. Also make constants?
    newMotor.burnFlash();
    return newMotor;
  }
  public void setIntake(double speed){
    IntakeMotorUp.set(speed);
    IntakeMotorDown.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("POWAHH (AMPS)", IntakeMotorUp.getOutputCurrent());
  }
}
