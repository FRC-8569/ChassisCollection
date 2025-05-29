package org.ZenithPolaris.Collections;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ZenithPolaris.Collections.Encoder.CTRECANCoder;
import org.ZenithPolaris.Collections.utils.DifferentialWheelConfig;
import org.ZenithPolaris.Collections.utils.DifferentialWheels;
import org.ZenithPolaris.Collections.utils.EncoderFrame;
import org.ZenithPolaris.Collections.utils.SwerveModuleConfig;
import org.ZenithPolaris.Collections.utils.SwerveMotor;
import org.ZenithPolaris.Collections.utils.SwerveMotorConfig;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Wheels {
    public static class SparkMaxWheel implements DifferentialWheels<SparkMax> {
        public SparkMax motor;
        public RelativeEncoder encoder;
        private SparkMaxConfig config;
        public DifferentialWheelConfig wheelConfig;

        public SparkMaxWheel(int MotorID, DifferentialWheelConfig wheelConfig) {
            motor = new SparkMax(MotorID, MotorType.kBrushless);
            config = new SparkMaxConfig();

            config
                    .idleMode(IdleMode.kBrake)
                    .inverted(wheelConfig.Inverted);

            config.encoder
                    .positionConversionFactor(1 / wheelConfig.GearRatio * wheelConfig.WheelCirc.in(Meters))
                    .velocityConversionFactor(1 / wheelConfig.GearRatio * wheelConfig.WheelCirc.in(Meters) / 60);

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            encoder = motor.getEncoder();
            this.wheelConfig = wheelConfig;
        }

        @Override
        public void setVelocity(LinearVelocity speed) {
            motor.set(speed.in(MetersPerSecond) / wheelConfig.MaxSpeed.in(MetersPerSecond));
        }

        @Override
        public Distance getPosition() {
            return Meters.of(encoder.getPosition());
        }

        @Override
        public LinearVelocity getVelocity() {
            return MetersPerSecond.of(encoder.getVelocity());
        }

        @Override
        public Temperature getMotorTemp() {
            return Celsius.of(motor.getMotorTemperature());
        }

        @Override
        public SparkMax getMotor() {
            return motor;
        }
    }

    public static class SparkFlexWheel implements DifferentialWheels<SparkFlex> {
        public SparkFlex motor;
        public RelativeEncoder encoder;
        private SparkFlexConfig config;
        public DifferentialWheelConfig wheelConfig;

        public SparkFlexWheel(int MotorID, DifferentialWheelConfig wheelConfig) {
            motor = new SparkFlex(MotorID, MotorType.kBrushless);
            config = new SparkFlexConfig();

            config
                    .idleMode(IdleMode.kBrake)
                    .inverted(wheelConfig.Inverted);

            config.encoder
                    .positionConversionFactor(1 / wheelConfig.GearRatio * wheelConfig.WheelCirc.in(Meters))
                    .velocityConversionFactor(1 / wheelConfig.GearRatio * wheelConfig.WheelCirc.in(Meters) / 60);

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            encoder = motor.getEncoder();
            this.wheelConfig = wheelConfig;
        }

        @Override
        public void setVelocity(LinearVelocity speed) {
            motor.set(speed.in(MetersPerSecond) / wheelConfig.MaxSpeed.in(MetersPerSecond));
        }

        @Override
        public Distance getPosition() {
            return Meters.of(encoder.getPosition());
        }

        @Override
        public LinearVelocity getVelocity() {
            return MetersPerSecond.of(encoder.getVelocity());
        }

        @Override
        public Temperature getMotorTemp() {
            return Celsius.of(motor.getMotorTemperature());
        }

        @Override
        public SparkFlex getMotor() {
            return motor;
        }
    }

    public static class TalonSRXWheel implements DifferentialWheels<TalonSRX> {
        public TalonSRX motor;
        public TalonSRXConfiguration config;
        public DifferentialWheelConfig wheelConfig;

        public TalonSRXWheel(int MotorID, DifferentialWheelConfig wheelConfig) {
            motor = new TalonSRX(MotorID);

            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
            motor.setInverted(wheelConfig.Inverted ? InvertType.InvertMotorOutput : InvertType.None);
            this.wheelConfig = wheelConfig;
        }

        @Override
        public void setVelocity(LinearVelocity speed) {
            motor.set(TalonSRXControlMode.PercentOutput,
                    speed.in(MetersPerSecond) / wheelConfig.MaxSpeed.in(MetersPerSecond));
        }

        @Override
        public LinearVelocity getVelocity() {
            return MetersPerSecond.of(motor.getMotorOutputPercent() * wheelConfig.MaxSpeed.in(MetersPerSecond));
        }

        @Override
        public Distance getPosition() {
            return Meters.of(motor.getSelectedSensorPosition() / 4096.0 * 360.0 * wheelConfig.WheelCirc.in(Meters)
                    / wheelConfig.GearRatio);
        }

        /**
         * The motor controller does not support temperature detecting
         */
        @Override
        public Temperature getMotorTemp() {
            return Celsius.of(0);
        }

        @Override
        public TalonSRX getMotor() {
            return motor;
        }
    }

    public static class TalonFXWheel implements DifferentialWheels<TalonFX> {
        public TalonFX motor;
        public DifferentialWheelConfig wheelConfig;
        public TalonFXConfiguration config;

        public TalonFXWheel(int MotorID, DifferentialWheelConfig wheelConfig) {
            motor = new TalonFX(MotorID);
            config = new TalonFXConfiguration();

            config.MotorOutput
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(wheelConfig.Inverted ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive);

            motor.getConfigurator().apply(config);
            this.wheelConfig = wheelConfig;
        }

        @Override
        public void setVelocity(LinearVelocity speed) {
            motor.set(speed.in(MetersPerSecond) / wheelConfig.MaxSpeed.in(MetersPerSecond));
        }

        @Override
        public Distance getPosition() {
            return Meters.of(motor.getPosition().getValue().in(Degrees) * wheelConfig.WheelCirc.in(Meters));
        }

        @Override
        public LinearVelocity getVelocity() {
            return MetersPerSecond
                    .of(motor.getVelocity().getValue().in(RotationsPerSecond) * wheelConfig.WheelCirc.in(Meters));
        }

        @Override
        public Temperature getMotorTemp() {
            return motor.getDeviceTemp().getValue();
        }

        @Override
        public TalonFX getMotor() {
            return motor;
        }
    }

    public class SwerveWheels{
        public enum MotorUsage{
            kDrive,
            kSteer;
        }
    
        public static class SparkMaxMotor implements SwerveMotor<SparkMax> {
            public SparkMax motor;
            public MotorUsage motorType;
            public RelativeEncoder encoder;
            private SparkMaxConfig config;
            public SwerveMotorConfig motorConfig;
            public SparkClosedLoopController DrivePID;

            public SparkMaxMotor(int MotorID, MotorUsage type, SwerveMotorConfig motorConfig){
                motor = new SparkMax(MotorID, MotorType.kBrushless);
                config = new SparkMaxConfig();
                config
                    .idleMode(IdleMode.kBrake)
                    .inverted(type == MotorUsage.kSteer);
                config.encoder
                    .positionConversionFactor(1/motorConfig.GearRatio*motorConfig.WheelCirc.in(Meters))
                    .velocityConversionFactor(1/motorConfig.GearRatio*motorConfig.WheelCirc.in(Meters)/60);

                config.closedLoop
                    .pidf(motorConfig.PID[0], motorConfig.PID[1], motorConfig.PID[2], motorConfig.PID[3])
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

                motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                this.motorConfig = motorConfig;
                encoder = motor.getEncoder();
                DrivePID = motor.getClosedLoopController();
            }

            @Override
            public void setSpeed(LinearVelocity speed){
                DrivePID.setReference(speed.in(MetersPerSecond), ControlType.kVelocity);
            }

            @Override
            public void setSpeedPercent(double speed){
                motor.set(speed);
            }

            @Override
            public Distance getPosition(){
                return Meters.of(encoder.getPosition());
            }

            @Override
            public LinearVelocity getVelocity(){
                return MetersPerSecond.of(encoder.getVelocity());
            }

            @Override
            public Temperature getTemp(){
                return Celsius.of(motor.getMotorTemperature());
            }

            @Override
            public SparkMax getMotor(){
                return motor;
            }
        }
    
        public static class KrakenMotor implements SwerveMotor<TalonFX> {
            public TalonFX motor;
            public TalonFXConfiguration config;
            public SwerveMotorConfig motorConfig;

            public KrakenMotor(int MotorID, MotorUsage type,SwerveMotorConfig motorConfig){
                motor = new TalonFX(MotorID);
                config = new TalonFXConfiguration();

                config.MotorOutput
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(type == MotorUsage.kSteer ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

                config.Slot0
                    .withKP(motorConfig.PID[0]).withKI(motorConfig.PID[1]).withKD(motorConfig.PID[2]).withKV(motorConfig.PID[3])
                    .withGravityType(GravityTypeValue.Elevator_Static);

                motor.getConfigurator().apply(config);
                this.motorConfig = motorConfig;
            }

            public KrakenMotor(int MotorID, MotorUsage type, SwerveMotorConfig motorConfig, TalonFXConfiguration configuration){
                motor = new TalonFX(MotorID);
                config = new TalonFXConfiguration();
                motor.getConfigurator().apply(configuration);
                this.motorConfig = motorConfig;
            }

            @Override
            public void setSpeed(LinearVelocity speed){
                motor.set(speed.in(MetersPerSecond)/motorConfig.MaxVelocity.in(MetersPerSecond));
            }
            
            @Override
            public void setSpeedPercent(double percent){
                motor.set(percent);
            }

            @Override
            public LinearVelocity getVelocity(){
                return MetersPerSecond.of(motor.getVelocity().getValue().in(RotationsPerSecond)*motorConfig.WheelCirc.in(Meters));
            }

            @Override
            public Distance getPosition(){
                return Meters.of(motor.getPosition().getValue().in(Rotations)*motorConfig.WheelCirc.in(Meters));
            }

            @Override
            public Temperature getTemp(){
                return motor.getDeviceTemp().getValue();
            }

            @Override
            public TalonFX getMotor(){
                return motor;
            }
        }
    
        public static class SwerveModules{
            public SwerveMotor<?> DriveMotor, SteerMotor;
            public EncoderFrame<?> encoder;
            private PIDController SteerPID;

            public SwerveModules(SwerveMotor<?> DriveMotor, SwerveMotor<?> SteerMotor, EncoderFrame<?> encoder, SwerveModuleConfig moduleConfig){
                this.DriveMotor = DriveMotor;
                this.SteerMotor = SteerMotor;
                this.encoder = encoder;
                SteerPID = new PIDController(moduleConfig.pid[0], moduleConfig.pid[1], moduleConfig.pid[2]);
            }

            public void setState(SwerveModuleState state){
                state.optimize(encoder.getRotation2d());

                DriveMotor.setSpeed(MetersPerSecond.of(state.speedMetersPerSecond));
                SteerMotor.setSpeedPercent(SteerPID.calculate(encoder.getPosition().in(Degrees), state.angle.getDegrees()));
            }

            public SwerveModuleState getState(){
                return new SwerveModuleState(
                    DriveMotor.getVelocity(),
                    encoder.getRotation2d()
                );
            }

            public SwerveModulePosition getPosition(){
                return new SwerveModulePosition(
                    DriveMotor.getPosition(),
                    encoder.getRotation2d()
                );
            }
            
        }
    }
}
