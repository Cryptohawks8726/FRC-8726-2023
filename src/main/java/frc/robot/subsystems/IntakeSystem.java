// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.RobotMap;

public class IntakeSystem extends SubsystemBase {
  private CANSparkMax extendIntakeMotor, intakePieceMotor;
  private DigitalInput intakeCollapseLimitSwitch, intakeExtendLimitSwitch;
  private boolean isCollapsed;
  
  public IntakeSystem() {
    extendIntakeMotor = new CANSparkMax(RobotMap.EXTEND_INTAKE_SPARKMAX, MotorType.kBrushless);
    intakePieceMotor = new CANSparkMax(RobotMap.INTAKE_BALL_SPARKMAX, MotorType.kBrushless);
    intakeCollapseLimitSwitch = new DigitalInput(RobotMap.INTAKE_COLLAPSE_LIMIT_SWITCH);
    intakeExtendLimitSwitch = new DigitalInput(RobotMap.INTAKE_EXTEND_LIMIT_SWITCH);  

    extendIntakeMotor.setIdleMode(IdleMode.kBrake);
    intakePieceMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleCollapse() {
    if (intakeExtendLimitSwitch.get() && intakeCollapseLimitSwitch.get()) {
      if (isCollapsed) {
        extendIntakeMotor.set(-0.15);
      }
      else {
        extendIntakeMotor.set(0.25);
      }
    }
    else {
      extendIntakeMotor.set(0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void switchIntakeState() {
    isCollapsed = !isCollapsed;
  }

  public void setExtendIntakeMotorVelocity(double speed) {
    extendIntakeMotor.set(-speed);
  }

  public void setIntakePieceMotorVelocity(double speed) {
    intakePieceMotor.set(-speed);
  }

  public boolean isCollapsed() {
    return isCollapsed;
  }

  public DigitalInput getIntakeCollapseLimitSwitch() {
    return intakeCollapseLimitSwitch;
  }

  public DigitalInput getIntakeExtendLimitSwitch() {
    return intakeExtendLimitSwitch;
  }
}
