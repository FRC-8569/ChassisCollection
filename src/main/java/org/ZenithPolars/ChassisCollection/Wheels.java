package org.ZenithPolars.ChassisCollection;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.Encoder;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.KOPWheelConfig;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.NormalDriveWheels;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.WheelSide;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.Swerve.MotorUsage;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.Swerve.SwerveModuleConfig;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.Swerve.SwerveWheels;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Wheels {
    public static class NormalWheels {
        public static class SparkMaxWheels implements NormalDriveWheels<SparkMax> {
            public SparkMax motor;
            public RelativeEncoder encoder;
            private SparkMaxConfig config;
            public KOPWheelConfig wheelConfig;

            public SparkMaxWheels(int MotorID, KOPWheelConfig wheelConfig) {
                motor = new SparkMax(MotorID, MotorType.kBrushless);
                encoder = motor.getEncoder();
                config = new SparkMaxConfig();

                config
                        .idleMode(IdleMode.kBrake)
                        .inverted(wheelConfig.side == WheelSide.RightSide);

                config.encoder
                        .positionConversionFactor(1 / wheelConfig.GearRatio * wheelConfig.WheelCirc.in(Meters))
                        .velocityConversionFactor(1 / wheelConfig.GearRatio * wheelConfig.WheelCirc.in(Meters) / 60);

                motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                this.wheelConfig = wheelConfig;
            }

            public SparkMaxWheels(int MotorID, KOPWheelConfig WheelConfig, SparkMaxConfig CustomConfig) {
                motor = new SparkMax(MotorID, MotorType.kBrushless);
                encoder = motor.getEncoder();
                motor.configure(CustomConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                this.wheelConfig = WheelConfig;
            }

            @Override
            public void setSpeed(LinearVelocity speed) {
                motor.set(speed.in(MetersPerSecond) / wheelConfig.maxSpeed.in(MetersPerSecond));
            }

            @Override
            public void setSpeed(double SpeedPercent) {
                motor.set(SpeedPercent);
            }

            @Override
            public void setSpeedMPS(double speed) {
                setSpeed(MetersPerSecond.of(speed));
            }

            @Override
            public LinearVelocity getVelocity() {
                return MetersPerSecond.of(encoder.getVelocity());
            }

            @Override
            public Distance getPosition() {
                return Meters.of(encoder.getPosition());
            }

            @Override
            public SparkMax getMotor() {
                return motor;
            }
        }

        public static class SparkFlexWheels implements NormalDriveWheels<SparkFlex> {
            public SparkFlex motor;
            public RelativeEncoder encoder;
            public SparkFlexConfig config;
            public KOPWheelConfig wheelConfig;

            public SparkFlexWheels(int MotorID, KOPWheelConfig wheelconfig) {
                motor = new SparkFlex(MotorID, MotorType.kBrushless);
                encoder = motor.getEncoder();
                config = new SparkFlexConfig();

                config
                        .idleMode(IdleMode.kBrake)
                        .inverted(wheelconfig.side == WheelSide.RightSide);

                config.encoder
                        .positionConversionFactor(1 / wheelConfig.GearRatio * wheelconfig.WheelCirc.in(Meters))
                        .velocityConversionFactor(1 / wheelConfig.GearRatio * wheelconfig.WheelCirc.in(Meters) / 60);

                motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                this.wheelConfig = wheelconfig;
            }

            public SparkFlexWheels(int MotorID, KOPWheelConfig wheelConfig, SparkFlexConfig CustomConfig) {
                motor = new SparkFlex(MotorID, MotorType.kBrushless);
                encoder = motor.getEncoder();

                motor.configure(CustomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                this.wheelConfig = wheelConfig;
            }

            @Override
            public void setSpeed(LinearVelocity speed) {
                motor.set(speed.in(MetersPerSecond) / wheelConfig.maxSpeed.in(MetersPerSecond));
            }

            @Override
            public void setSpeed(double SpeedPercent) {
                motor.set(SpeedPercent);
            }

            @Override
            public void setSpeedMPS(double speed) {
                setSpeed(MetersPerSecond.of(speed));
            }

            @Override
            public LinearVelocity getVelocity() {
                return MetersPerSecond.of(encoder.getVelocity());
            }

            @Override
            public Distance getPosition() {
                return Meters.of(encoder.getPosition());
            }

            @Override
            public SparkFlex getMotor() {
                return motor;
            }
        }

        public static class TalonSRXWheels implements NormalDriveWheels<TalonSRX> {
            public TalonSRX motor;
            public KOPWheelConfig wheelConfig;

            public TalonSRXWheels(int MotorID, KOPWheelConfig wheelConfig) {
                motor = new TalonSRX(MotorID);
                motor.configFactoryDefault();
                motor.setNeutralMode(NeutralMode.Brake);
                motor.setInverted(
                        wheelConfig.side == WheelSide.RightSide ? InvertType.InvertMotorOutput : InvertType.None);
                motor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 500);
            }

            @Override
            public void setSpeed(LinearVelocity speed) {
                motor.set(ControlMode.PercentOutput,
                        speed.in(MetersPerSecond) / wheelConfig.maxSpeed.in(MetersPerSecond));
            }

            @Override
            public void setSpeed(double percent) {
                motor.set(ControlMode.PercentOutput, percent);
            }

            @Override
            public void setSpeedMPS(double speed) {
                setSpeed(MetersPerSecond.of(speed));
            }

            @Override
            public LinearVelocity getVelocity() {
                return MetersPerSecond
                        .of(motor.getSelectedSensorVelocity() / 4096.0 / 60 * wheelConfig.WheelCirc.in(Meters));
            }

            @Override
            public Distance getPosition() {
                return Meters.of(motor.getSelectedSensorPosition() / 4096.0 * wheelConfig.WheelCirc.in(Meters));
            }

            @Override
            public TalonSRX getMotor() {
                return motor;
            }
        }

        public static class TalonFXSWheels implements NormalDriveWheels<TalonFXS> {
            public TalonFXS motor;
            private TalonFXSConfiguration configuration;
            public KOPWheelConfig wheelConfig;

            public TalonFXSWheels(int MotorID, KOPWheelConfig wheelConfig) {
                motor = new TalonFXS(MotorID);
                configuration = new TalonFXSConfiguration();

                configuration.MotorOutput
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(wheelConfig.side == WheelSide.RightSide ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);

                motor.getConfigurator().apply(configuration);
                this.wheelConfig = wheelConfig;
            }

            @Override
            public void setSpeed(LinearVelocity speed) {
                motor.set(speed.in(MetersPerSecond) / wheelConfig.maxSpeed.in(MetersPerSecond));
            }

            @Override
            public void setSpeed(double speed) {
                motor.set(speed);
            }

            @Override
            public void setSpeedMPS(double speed) {
                setSpeed(MetersPerSecond.of(speed));
            }

            @Override
            public LinearVelocity getVelocity() {
                return MetersPerSecond
                        .of(motor.getVelocity().getValue().in(RotationsPerSecond) * wheelConfig.WheelCirc.in(Meters));
            }

            @Override
            public Distance getPosition() {
                return Meters.of(motor.getPosition().getValue().in(Rotation) * wheelConfig.WheelCirc.in(Meters));
            }

            @Override
            public TalonFXS getMotor() {
                return motor;
            }

        }

        public static class TalonFXWheels implements NormalDriveWheels<TalonFX> {
            public TalonFX motor;
            private TalonFXConfiguration configuration;
            public KOPWheelConfig wheelConfig;

            public TalonFXWheels(int MotorID, KOPWheelConfig wheelConfig) {
                motor = new TalonFX(MotorID);
                configuration = new TalonFXConfiguration();

                configuration.MotorOutput
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(wheelConfig.side == WheelSide.RightSide ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive);

                motor.getConfigurator().apply(configuration);
                this.wheelConfig = wheelConfig;
            }

            @Override
            public void setSpeed(LinearVelocity speed) {
                motor.set(speed.in(MetersPerSecond) / wheelConfig.maxSpeed.in(MetersPerSecond));
            }

            @Override
            public void setSpeed(double speed) {
                motor.set(speed);
            }

            @Override
            public void setSpeedMPS(double speed) {
                setSpeed(MetersPerSecond.of(speed));
            }

            @Override
            public LinearVelocity getVelocity() {
                return MetersPerSecond
                        .of(motor.getVelocity().getValue().in(RotationsPerSecond) * wheelConfig.WheelCirc.in(Meters));
            }

            @Override
            public Distance getPosition() {
                return Meters.of(motor.getPosition().getValue().in(Rotation) * wheelConfig.WheelCirc.in(Meters));
            }

            @Override
            public TalonFX getMotor() {
                return motor;
            }

        }
    }

    public static class Swerve {
        public static class SparkMaxWheels implements SwerveWheels<SparkMax> {
            public SparkMax motor;
            public RelativeEncoder encoder;
            public SparkMaxConfig config;
            public SparkClosedLoopController pid;

            public SparkMaxWheels(int MotorID, MotorUsage usage, SwerveModuleConfig moduleConfig) {
                motor = new SparkMax(MotorID, MotorType.kBrushless);
                config = new SparkMaxConfig();

                config
                        .idleMode(IdleMode.kBrake)
                        .inverted(usage == MotorUsage.Steer);
                config.encoder
                        .positionConversionFactor(1 / moduleConfig.GearRatio * moduleConfig.WheelCirc.in(Meters))
                        .velocityConversionFactor(1 / moduleConfig.GearRatio * moduleConfig.WheelCirc.in(Meters));

                if (usage == MotorUsage.Drive)
                    config.closedLoop.pidf(moduleConfig.DrivePID[0], moduleConfig.DrivePID[1], moduleConfig.DrivePID[2],
                            moduleConfig.DrivePID[3]);

                motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                encoder = motor.getEncoder();
                pid = motor.getClosedLoopController();
            }

            @Override
            public void setSpeed(double speedMPS) {
                pid.setReference(speedMPS, ControlType.kVelocity);
            }

            @Override
            public void setSpeedPerc(double percent) {
                motor.set(percent);
            }

            @Override
            public LinearVelocity getVelocity() {
                return MetersPerSecond.of(encoder.getVelocity());
            }

            @Override
            public Distance getPosition() {
                return Meters.of(encoder.getPosition());
            }

            @Override
            public SparkMax getMotor() {
                return motor;
            }
        }

        public static class SparkFlexWheels implements SwerveWheels<SparkFlex> {
            public SparkFlex motor;
            public SparkFlexConfig config;
            public RelativeEncoder encoder;
            public SparkClosedLoopController pid;
            public SwerveModuleConfig moduleConfig;

            public SparkFlexWheels(int MotorID, MotorUsage usage, SwerveModuleConfig moduleConfig) {
                motor = new SparkFlex(MotorID, MotorType.kBrushless);
                config = new SparkFlexConfig();

                config
                        .idleMode(IdleMode.kBrake)
                        .inverted(usage == MotorUsage.Steer);
                config.encoder
                        .positionConversionFactor(1 / moduleConfig.GearRatio * moduleConfig.WheelCirc.in(Meters))
                        .velocityConversionFactor(1 / moduleConfig.GearRatio * moduleConfig.WheelCirc.in(Meters));

                if (usage == MotorUsage.Drive)
                    config.closedLoop.pidf(moduleConfig.DrivePID[0], moduleConfig.DrivePID[1], moduleConfig.DrivePID[2],
                            moduleConfig.DrivePID[3]);

                motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                encoder = motor.getEncoder();
                pid = motor.getClosedLoopController();
            }

            @Override
            public void setSpeed(double speedMPS) {
                pid.setReference(speedMPS, ControlType.kVelocity);
            }

            @Override
            public void setSpeedPerc(double percent) {
                motor.set(percent);
            }

            @Override
            public LinearVelocity getVelocity() {
                return MetersPerSecond.of(encoder.getVelocity());
            }

            @Override
            public Distance getPosition() {
                return Meters.of(encoder.getPosition());
            }

            @Override
            public SparkFlex getMotor() {
                return motor;
            }
        }

        public static class TalonFXSWheels implements SwerveWheels<TalonFXS> {
            public TalonFXS motor;
            public TalonFXSConfiguration configuration;
            public VelocityVoltage pid;
            public Distance WheelCirc;

            public TalonFXSWheels(int MotorID, MotorUsage usage, SwerveModuleConfig moduleConfig) {
                motor = new TalonFXS(MotorID);
                configuration = new TalonFXSConfiguration();

                configuration.MotorOutput
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(usage == MotorUsage.Steer ? InvertedValue.CounterClockwise_Positive
                                : InvertedValue.Clockwise_Positive);

                motor.getConfigurator().apply(configuration);
                this.WheelCirc = moduleConfig.WheelCirc;
            }

            public TalonFXSWheels(int MotorID, MotorUsage usage, SwerveModuleConfig moduleConfig,
                    Slot0Configs CustomPID) {
                motor = new TalonFXS(MotorID);
                configuration = new TalonFXSConfiguration();

                configuration.MotorOutput
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(usage == MotorUsage.Steer ? InvertedValue.CounterClockwise_Positive
                                : InvertedValue.Clockwise_Positive);

                configuration.withSlot0(CustomPID);
                motor.getConfigurator().apply(configuration);
                pid = new VelocityVoltage(0).withSlot(0);
            }

            @Override
            public void setSpeed(double speedMPS) {
                motor.setControl(pid.withVelocity(RotationsPerSecond.of(speedMPS / WheelCirc.in(Meters))));
            }

            @Override
            public void setSpeedPerc(double percent) {
                motor.set(percent);
            }

            @Override
            public LinearVelocity getVelocity() {
                return MetersPerSecond.of(motor.getVelocity().getValue().in(RotationsPerSecond) * WheelCirc.in(Meters));
            }

            @Override
            public Distance getPosition() {
                return Meters.of(motor.getPosition().getValue().in(Rotations) * WheelCirc.in(Meters));
            }

            @Override
            public TalonFXS getMotor() {
                return motor;
            }
        }

        public static class TalonFXWheels implements SwerveWheels<TalonFX> {
            public TalonFX motor;
            public TalonFXConfiguration configuration;
            public VelocityVoltage pid;
            public Distance WheelCirc;

            public TalonFXWheels(int MotorID, MotorUsage usage, SwerveModuleConfig moduleConfig) {
                motor = new TalonFX(MotorID);
                configuration = new TalonFXConfiguration();

                configuration.MotorOutput
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(usage == MotorUsage.Steer ? InvertedValue.CounterClockwise_Positive
                                : InvertedValue.Clockwise_Positive);

                motor.getConfigurator().apply(configuration);
                this.WheelCirc = moduleConfig.WheelCirc;
            }

            public TalonFXWheels(int MotorID, MotorUsage usage, SwerveModuleConfig moduleConfig,
                    Slot0Configs CustomPID) {
                motor = new TalonFX(MotorID);
                configuration = new TalonFXConfiguration();

                configuration.MotorOutput
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(usage == MotorUsage.Steer ? InvertedValue.CounterClockwise_Positive
                                : InvertedValue.Clockwise_Positive);

                configuration.withSlot0(CustomPID);
                motor.getConfigurator().apply(configuration);
                pid = new VelocityVoltage(0).withSlot(0);
            }

            @Override
            public void setSpeed(double speedMPS) {
                motor.setControl(pid.withVelocity(RotationsPerSecond.of(speedMPS / WheelCirc.in(Meters))));
            }

            @Override
            public void setSpeedPerc(double percent) {
                motor.set(percent);
            }

            @Override
            public LinearVelocity getVelocity() {
                return MetersPerSecond.of(motor.getVelocity().getValue().in(RotationsPerSecond) * WheelCirc.in(Meters));
            }

            @Override
            public Distance getPosition() {
                return Meters.of(motor.getPosition().getValue().in(Rotations) * WheelCirc.in(Meters));
            }

            @Override
            public TalonFX getMotor() {
                return motor;
            }
        }

        public static class SwerveModule {
            public SwerveWheels<?> DriveMotor, SteerMotor;
            public Encoder<?> encoder;
            public PIDController SteerPID;

            public SwerveModule(SwerveWheels<?> DriveMotor, SwerveWheels<?> SteerMotors, Encoder<?> encoder, SwerveModuleConfig moduleConfig){
                this.DriveMotor = DriveMotor;
                this.SteerMotor = SteerMotors;
                this.encoder = encoder;
                this.SteerPID = new PIDController(moduleConfig.SteerPID[0], moduleConfig.SteerPID[1], moduleConfig.SteerPID[2]);
            }

            public void setState(SwerveModuleState state){
                state.optimize(getState().angle);

                DriveMotor.setSpeed(state.speedMetersPerSecond);
                SteerMotor.setSpeedPerc(SteerPID.calculate(getState().angle.getDegrees(), state.angle.getDegrees()));
            }
            
            public SwerveModuleState getState(){
                return new SwerveModuleState(
                    DriveMotor.getVelocity(),
                    Rotation2d.fromDegrees(encoder.getAngle().in(Degrees))
                );
            }

            public SwerveModulePosition getPosition(){
                return new SwerveModulePosition(
                    DriveMotor.getPosition(),
                    Rotation2d.fromDegrees(encoder.getAngle().in(Degrees))
                );
            }
        }
    }
}
