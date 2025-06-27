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

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */

// TODO: later, add sim classes. focus on functionality and think about code structure first.
public class ElevatorIOTalonFX implements ElevatorIO {

  // Hardware objects
  private final TalonFX master;
  private final TalonFX follower;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);
  private final CoastOut coastRequest = new CoastOut();
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  // Timestamp inputs from Phoenix thread
  // Inputs to log from elevator motors
  private final StatusSignal<Angle> masterPosition;
  private final StatusSignal<AngularVelocity> masterVelocity;
  private final StatusSignal<Voltage> masterAppliedVolts;
  private final StatusSignal<Current> masterSupplyCurrentAmps;
  private final StatusSignal<Current> masterTorqueCurrentAmps;
  private final StatusSignal<Temperature> masterTemp;

  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerSupplyCurrentAmps;
  private final StatusSignal<Current> followerTorqueCurrentAmps;
  private final StatusSignal<Temperature> followerTemp;

  public ElevatorIOTalonFX() {
    master = new TalonFX(Constants.RobotMap.CAN.ELEVATOR_MASTER);
    follower = new TalonFX(Constants.RobotMap.CAN.ELEVATOR_FOLLOWER);

    var elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 8;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // TESTING ONLY
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // TESTING ONLY
    // masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 54;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

    elevatorConfig.CurrentLimits.StatorCurrentLimit = 120;
    elevatorConfig.Feedback.SensorToMechanismRatio = 12;
    elevatorConfig.Voltage.PeakForwardVoltage = 16;
    elevatorConfig.Voltage.PeakReverseVoltage = -16;

    elevatorConfig.Slot0.kP = 18;
    elevatorConfig.Slot0.kI = 2;
    elevatorConfig.Slot0.kD = 2;
    elevatorConfig.Slot0.kG = 0.3;
    elevatorConfig.Slot0.kS = 0.1;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 45;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 110;

    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Configure motors
    tryUntilOk(5, () -> master.getConfigurator().apply(elevatorConfig, 0.25));
    tryUntilOk(5, () -> master.getConfigurator().apply(elevatorConfig, 0.25));
    tryUntilOk(5, () -> follower.setPosition(0.0));
    tryUntilOk(5, () -> master.setPosition(0.0));

    masterPosition = master.getPosition();
    masterVelocity = master.getVelocity();
    masterAppliedVolts = master.getMotorVoltage();
    masterSupplyCurrentAmps = master.getSupplyCurrent();
    masterTorqueCurrentAmps = master.getTorqueCurrent();
    masterTemp = master.getDeviceTemp();

    followerPosition = follower.getPosition();
    followerVelocity = follower.getVelocity();
    followerAppliedVolts = follower.getMotorVoltage();
    followerSupplyCurrentAmps = follower.getSupplyCurrent();
    followerTorqueCurrentAmps = follower.getTorqueCurrent();
    followerTemp = follower.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        masterPosition,
        masterVelocity,
        masterAppliedVolts,
        masterSupplyCurrentAmps,
        masterTorqueCurrentAmps,
        masterTemp,
        followerPosition,
        followerVelocity,
        followerAppliedVolts,
        followerSupplyCurrentAmps,
        followerTorqueCurrentAmps,
        followerTemp);
    master.optimizeBusUtilization();
    follower.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false,
        masterPosition,
        masterVelocity,
        masterAppliedVolts,
        masterSupplyCurrentAmps,
        masterTorqueCurrentAmps,
        masterTemp,
        followerPosition,
        followerVelocity,
        followerAppliedVolts,
        followerSupplyCurrentAmps,
        followerTorqueCurrentAmps,
        followerTemp);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Refresh all signals
    inputs.data =
        new ElevatorIOData(
            BaseStatusSignal.isAllGood(
                masterPosition,
                masterVelocity,
                masterAppliedVolts,
                masterSupplyCurrentAmps,
                masterTorqueCurrentAmps,
                masterTemp),
            BaseStatusSignal.isAllGood(
                followerPosition,
                followerVelocity,
                followerAppliedVolts,
                followerSupplyCurrentAmps,
                followerTorqueCurrentAmps,
                followerTemp),
            masterPosition.getValue().in(Radians),
            followerPosition.getValue().in(Radians),
            masterVelocity.getValue().in(RadiansPerSecond),
            followerVelocity.getValue().in(RadiansPerSecond),
            masterAppliedVolts.getValueAsDouble(),
            followerAppliedVolts.getValueAsDouble(),
            masterTorqueCurrentAmps.getValueAsDouble(),
            followerTorqueCurrentAmps.getValueAsDouble(),
            masterSupplyCurrentAmps.getValueAsDouble(),
            followerSupplyCurrentAmps.getValueAsDouble(),
            masterTemp.getValueAsDouble(),
            followerTemp.getValueAsDouble());
  }

  @Override
  public void runVolts(double volts) {
    master.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void runPosition(double position) {
    master.setControl(magicRequest.withPosition(position));
  }

  @Override
  public void stop() {
    master.stopMotor();
  }

  @Override
  public void coast() {
    master.setControl(coastRequest);
  }

  @Override
  public void targetLastPosition() {
    master.setControl(positionRequest.withPosition(master.getPosition().getValueAsDouble()));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    tryUntilOk(
        5,
        () -> master.getConfigurator().apply(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD)));
    tryUntilOk(
        5,
        () ->
            follower.getConfigurator().apply(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD)));
  }

  @Override
  public void setCharacterization(double kA, double kV, double kS, double kG) {
    tryUntilOk(
        5,
        () ->
            master
                .getConfigurator()
                .apply(new Slot0Configs().withKA(kA).withKV(kV).withKS(kS).withKG(kG)));
    tryUntilOk(
        5,
        () ->
            follower
                .getConfigurator()
                .apply(new Slot0Configs().withKA(kA).withKV(kV).withKS(kS).withKG(kG)));
  }

  @Override
  public void resetPosition() {
    tryUntilOk(5, () -> master.setPosition(0.0));
    tryUntilOk(5, () -> follower.setPosition(0.0));
  }
}
