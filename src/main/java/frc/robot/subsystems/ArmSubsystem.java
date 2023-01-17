// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;

  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR_SPARKMAX, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setArmPower(double speed){
    armMotor.set(speed);
  }
}
