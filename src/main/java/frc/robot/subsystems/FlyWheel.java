// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import frc.robot.Constants.FlyWheelConstants;
import frc.util.NetworkTables;

public class FlyWheel extends SubsystemBase {
  /** Creates a new FlyWheel. */
  private final TalonFXS flyMotor = new TalonFXS(FlyWheelConstants.flyWheelID);

  private TalonFXSConfiguration talonFXSConfigs;
  private Slot0Configs slot0Configs;
  private MotionMagicConfigs motionMagicConfigs;

  public FlyWheel() {
    ConfigureMotor();
  }

  public void ConfigureMotor() {
    talonFXSConfigs = new TalonFXSConfiguration();

    slot0Configs = talonFXSConfigs.Slot0;
    slot0Configs.kS = NetworkTables.FlyTable.kS.get(); // Add 0.25 V output to overcome static friction
    slot0Configs.kV = NetworkTables.FlyTable.kV.get(); // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = NetworkTables.FlyTable.kA.get(); // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = NetworkTables.FlyTable.kP.get(); // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = NetworkTables.FlyTable.kI.get(); // no output for integrated error
    slot0Configs.kD = NetworkTables.FlyTable.kD.get(); // no output for error derivative
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    // set Motion Magic settings
    motionMagicConfigs = talonFXSConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = NetworkTables.FlyTable.MotionMagicAcceleration.get(); // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = NetworkTables.FlyTable.MotionMagicJerk.get(); // Target jerk of 4000 rps/s/s (0.1 seconds)
    flyMotor.getConfigurator().apply(talonFXSConfigs);
  }

  public void setFlyWheelVelocity(double shooterMotorPower) {
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    //leftShooterMotor.setControl(m_request.withVelocity(shooterMotorPower));
    System.out.println(m_request.withVelocity(shooterMotorPower) + "\n");
    flyMotor.setControl(m_request.withVelocity(shooterMotorPower));    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTables.FlyTable.velocity.set(flyMotor.getVelocity().getValueAsDouble());
  }
}
