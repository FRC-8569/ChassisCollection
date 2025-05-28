package ZenithPolaris.Collections.KOPChassis;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ejml.masks.Mask;
import org.opencv.core.Scalar;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import ZenithPolaris.Collections.utils.DifferentialDrive;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;

public class Wheels {
    public class SparkMaxWheels implements DifferentialDrive.DifferentialWheel<SparkMax>{
        public SparkMax motor;
        public RelativeEncoder encoder;
        private SparkMaxConfig config;
        public LinearVelocity MaxSpeed;

        /**
         * 
         * @param MotorID is the unique CAN ID of the Motor 
         * @param WheelConfig is the configuration of the wheel which can found in {@link DifferentialDrive.WheelConfigraion}
         */
        public SparkMaxWheels(int MotorID, DifferentialDrive.WheelConfigraion WheelConfig, LinearVelocity MaxSpeed){
            motor = new SparkMax(MotorID, MotorType.kBrushless);
            config = new SparkMaxConfig();

            config
                .idleMode(IdleMode.kBrake)
                .inverted(WheelConfig.inverted);

            config.encoder
                .positionConversionFactor(1/WheelConfig.GearRatio*WheelConfig.WheelSize.in(Meters)*Math.PI)
                .velocityConversionFactor(1/WheelConfig.GearRatio*WheelConfig.WheelSize.in(Meters)*Math.PI/60);

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            encoder = motor.getEncoder();
            this.MaxSpeed = MaxSpeed;
        }

        /**
         * 
         * @param MotorID is the unique CAN ID of the motor
         * @param WheelConfig is the configuration of the wheel which can found in {@link DifferentialDrive.WheelConfigraion}
         * @param custConfig is the custom motor configuration of your motor
         */
        public SparkMaxWheels(int MotorID, DifferentialDrive.WheelConfigraion WheelConfig, SparkMaxConfig custConfig, LinearVelocity MaxSpeed){
            motor = new SparkMax(MotorID, MotorType.kBrushless);

            custConfig
                .idleMode(IdleMode.kBrake)
                .inverted(WheelConfig.inverted);

            custConfig.encoder
                .positionConversionFactor(1/WheelConfig.GearRatio*WheelConfig.WheelSize.in(Meters)*Math.PI)
                .velocityConversionFactor(1/WheelConfig.GearRatio*WheelConfig.WheelSize.in(Meters)*Math.PI/60);

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            encoder = motor.getEncoder();
            this.MaxSpeed = MaxSpeed;
        }

        @Override
        public void setVelocity(LinearVelocity speed){
            motor.set(speed.in(MetersPerSecond)/MaxSpeed.in(MetersPerSecond));
        }

        @Override
        public LinearVelocity getVelocity(){
            return MetersPerSecond.of(encoder.getVelocity());
        }

        @Override
        public Distance getPosition(){
            return Meters.of(encoder.getPosition());
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

    public class SparkFlexWheels implements DifferentialDrive.DifferentialWheel<SparkFlex>{
        public SparkFlex motor;
        public RelativeEncoder encoder;
        public SparkFlexConfig config;
        public LinearVelocity MaxSpeed;
        
        /**
         * @param MotorID is the unique CAN ID of the motor
         * @param WheelConfig is the configuration of the wheel which can found in {@link DifferentialDrive.WheelConfigraion}
         */
        public SparkFlexWheels(int MotorID, DifferentialDrive.WheelConfigraion WheelConfig, LinearVelocity MaxSpeed){
            motor = new SparkFlex(MotorID, MotorType.kBrushless);
            config = new SparkFlexConfig();

            config
                .idleMode(IdleMode.kBrake)
                .inverted(WheelConfig.inverted);

            config.encoder
                .positionConversionFactor(1/WheelConfig.GearRatio*WheelConfig.WheelSize.in(Meters)*Math.PI)
                .velocityConversionFactor(1/WheelConfig.GearRatio*WheelConfig.WheelSize.in(Meters)*Math.PI/60);

            motor.configure(config, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
            encoder = motor.getEncoder();
            
            this.MaxSpeed = MaxSpeed;
        }
        
        /**
         * 
         * @param MotorID is the unique CAN ID of the motor
         * @param WheelConfig is the configuration of the wheel which can found in {@link DifferentialDrive.WheelConfigraion}
         * @param custConfig is the custom motor configuration
         */
        public SparkFlexWheels(int MotorID, DifferentialDrive.WheelConfigraion WheelConfig, SparkFlexConfig custConfig, LinearVelocity MaxSpeed){
            motor = new SparkFlex(MotorID, MotorType.kBrushless);
            config = new SparkFlexConfig();

            custConfig
                .idleMode(IdleMode.kBrake)
                .inverted(WheelConfig.inverted);

            custConfig.encoder
                .positionConversionFactor(1/WheelConfig.GearRatio*WheelConfig.WheelSize.in(Meters)*Math.PI)
                .velocityConversionFactor(1/WheelConfig.GearRatio*WheelConfig.WheelSize.in(Meters)*Math.PI/60);

            motor.configure(custConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
            encoder = motor.getEncoder();
            this.MaxSpeed = MaxSpeed;
            }
        
        @Override
        public void setVelocity(LinearVelocity speed){
            motor.set(speed.in(MetersPerSecond)/MaxSpeed.in(MetersPerSecond));
        }

        @Override
        public LinearVelocity getVelocity(){
            return MetersPerSecond.of(encoder.getVelocity());
        }

        @Override
        public Distance getPosition(){
            return Meters.of(encoder.getPosition());
        }

        @Override
        public Temperature getTemp(){
            return Celsius.of(motor.getMotorTemperature());
        }

        @Override
        public SparkFlex getMotor(){
            return motor;
        } 
    }

    public class TalonFXWheels implements DifferentialDrive.DifferentialWheel<TalonFX> {
        public TalonFX motor;
        public TalonFXConfiguration config;
        public LinearVelocity MaxSpeed;
        public Distance WheelSize;

        /**
         * 
         * @param MotorID is the unique CAN ID of your motor which can found in Phoenix Tuner X
         * @param WheelConfig is the configuration of the wheel in {@link DifferentialDrive.WheelConfigraion}
         */
        public TalonFXWheels(int MotorID, DifferentialDrive.WheelConfigraion WheelConfig, LinearVelocity MaxSpeed){
            motor = new TalonFX(MotorID);

            config = new TalonFXConfiguration();

            config.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(WheelConfig.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

            motor.getConfigurator().apply(config);

            MaxSpeed = MetersPerSecond.of(5676/WheelConfig.GearRatio*WheelConfig.WheelSize.in(Meters)*Math.PI);
            motor.getConfigurator().apply(config);
            WheelSize = WheelConfig.WheelSize;
            this.MaxSpeed = MaxSpeed;
        }

        /**
         * 
         * @param MotorID is the unique CAN ID of your motor which can found in Phoenix Tuner X
         * @param WheelConfig is the configuration of the wheel in {@link DifferentialDrive.WheelConfigraion}
         * @param configuration is the custom configuration of the motor
         */
        public TalonFXWheels(int MotorID, DifferentialDrive.WheelConfigraion WheelConfig, TalonFXConfiguration configuration, LinearVelocity MaxSpeed){
            motor = new TalonFX(MotorID);

            configuration.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(WheelConfig.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

            configuration.Feedback
                .withSensorToMechanismRatio(WheelConfig.GearRatio);

            motor.getConfigurator().apply(config);

            MaxSpeed = MetersPerSecond.of(5676/WheelConfig.GearRatio*WheelConfig.WheelSize.in(Meters)*Math.PI);
            motor.getConfigurator().apply(configuration);
            WheelSize = WheelConfig.WheelSize;
            this.MaxSpeed = MaxSpeed;
        }

        @Override
        public void setVelocity(LinearVelocity speed){
            motor.set(speed.in(MetersPerSecond)/MaxSpeed.in(MetersPerSecond));
        }

        @Override
        public LinearVelocity getVelocity(){
            return MetersPerSecond.of(motor.getVelocity().getValueAsDouble()*WheelSize.in(Meters)*Math.PI);
        }

        @Override
        public Distance getPosition(){
            return Meters.of(motor.getPosition().getValueAsDouble()*WheelSize.in(Meters)*Math.PI);
        }
        
        @Override
        public Temperature getTemp(){
            return Celsius.of(motor.getDeviceTemp().getValueAsDouble());
        }

        @Override
        public TalonFX getMotor(){
            return motor;
        }
    }

}
