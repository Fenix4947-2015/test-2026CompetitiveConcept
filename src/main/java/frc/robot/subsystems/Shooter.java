package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Shooter extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double dashboardTargetRPM = 0.0;

    public Shooter() {
        leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANBus);
        middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANBus);
        rightMotor = new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANBus);
        motors = List.of(leftMotor, middleMotor, rightMotor);

        configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(middleMotor, InvertedValue.Clockwise_Positive);
        configureMotor(rightMotor, InvertedValue.Clockwise_Positive);

        SmartDashboard.putData(this);
    }

    /**
     * Configures a TalonFX motor with inversion, voltage, current limits, and PID settings.
     * @param motor The TalonFX motor to configure.
     * @param invertDirection The direction to invert the motor.
     */
    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }

    /**
     * Sets all shooter motors to a target RPM using velocity control.
     * Intent: Achieve and maintain a desired shooter wheel speed for consistent shooting.
     * @param rpm The desired RPM to set.
     */
    public void setRPM(double rpm) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(rpm))
            );
        }
    }

    /**
     * Sets all shooter motors to a percent output (open loop voltage control).
     * Intent: Provide open-loop control for testing or manual operation.
     * @param percentOutput The percent output (-1.0 to 1.0).
     */
    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                voltageRequest
                    .withOutput(Volts.of(percentOutput * 12.0))
            );
        }
    }

    /**
     * Stops all shooter motors by setting percent output to zero.
     * Intent: Quickly halt the shooter for safety or control.
     */
    public void stop() {
        setPercentOutput(0.0);
    }

    /**
     * Returns a command to spin up the shooter to a target RPM and wait until velocity is within tolerance.
     * Intent: Automate shooter spin-up and readiness check for firing.
     * @param rpm The target RPM.
     * @return The spin-up command.
     */
    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    /**
     * Returns a command to spin up the shooter to the RPM set on the dashboard.
     * Intent: Allow operators to set shooter speed dynamically from the dashboard.
     * @return The dashboard spin-up command.
     */
    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM)); 
    }

    /**
     * Checks if all shooter motors are in velocity mode and within the velocity tolerance.
     * Intent: Confirm shooter is ready to fire with consistent speed.
     * @return True if all motors are within tolerance, false otherwise.
     */
    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }

    /**
     * Adds motor telemetry properties to the SendableBuilder for SmartDashboard.
     * Intent: Provide real-time feedback for monitoring and debugging.
     * @param builder The SendableBuilder.
     * @param motor The motor to add properties for.
     * @param name The name prefix for properties.
     */
    private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }

    /**
     * Initializes SmartDashboard properties for all shooter motors and subsystem state.
     * Intent: Enable operator visibility and control of shooter state.
     * @param builder The SendableBuilder.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, leftMotor, "Left");
        initSendable(builder, middleMotor, "Middle");
        initSendable(builder, rightMotor, "Right");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
    }
}
