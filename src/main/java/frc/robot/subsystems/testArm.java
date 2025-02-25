// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class testArm extends SubsystemBase {
  private SparkMax motorA = new SparkMax(12, MotorType.kBrushless);

  private SparkClosedLoopController controllerA = motorA.getClosedLoopController();

  private double setPoint = 0;

  private RelativeEncoder encoderA = motorA.getEncoder();
  

  /** Creates a new testArm. */
  public testArm() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);
    config.closedLoop.pid(0.01, 0, 0.002);

    motorA.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  public void setArmPose(double setPoint) {
    this.setPoint = setPoint;
    controllerA.setReference(setPoint, SparkBase.ControlType.kPosition);
  }

  public boolean isAtSetPoint() {
    if((setPoint - 1 < encoderA.getPosition()) && (encoderA.getPosition() < setPoint + 1))
      return true;
    else 
      return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmPoseA", encoderA.getPosition());
    SmartDashboard.putNumber("Arm setPoint", setPoint);
    SmartDashboard.putBoolean("ArmAtSetPoint", isAtSetPoint());
  }
}
