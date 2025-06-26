// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.coral;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */

// TODO: later, add sim classes. focus on functionality and think about code structure first.
public class CoralIOTalonSRX implements CoralIO {

  // Hardware objects
  private final TalonSRX motor;
  private final AnalogInput irSensor;

  // Inputs to log
  double motorOutput;
  double stator;
  double irVoltage;

  public CoralIOTalonSRX() {
    motor = new TalonSRX(0);
    irSensor = new AnalogInput(0);

    // Configure system
    motor.configFactoryDefault();
    irSensor.setOversampleBits(4);
    irSensor.setAverageBits(4);

    motorOutput = motor.getMotorOutputPercent();
    motorOutput = motor.getStatorCurrent();
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    // Refresh all signals
    inputs.data =
        new CoralIOData(
            motor.getMotorOutputPercent(), irSensor.getAverageVoltage(), motor.getStatorCurrent());
  }

  @Override
  public void setOutput(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }
}
