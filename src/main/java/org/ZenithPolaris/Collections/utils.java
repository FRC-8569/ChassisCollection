package org.ZenithPolaris.Collections;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;

public class utils {
    public interface DifferentialWheels<M> {
        /**
         * 
         * @param speed with setting the velocity in {@code LinearVelocity}
         */
        void setVelocity(LinearVelocity speed);
        
        /**
         * 
         * @return the motor position
         */
        Distance getPosition();

        /**
         * 
         * @return returns the motor veloicty in percent
         */
        LinearVelocity getVelocity();

        /**
         * 
         * @return the motor temperautre
         */
        Temperature getMotorTemp();

        M getMotor();
    }

    public static class DifferentialWheelConfig {
        public Distance WheelCirc = Meters.of(1);
        public LinearVelocity MaxSpeed = MetersPerSecond.of(1);
        public boolean Inverted = false;
        public double GearRatio = 1;

        /**
         * This is the configuration of the wheel
         */
        public DifferentialWheelConfig(){}

        /**
         * 
         * @param diameter is the wheel diamaters
         * @return itself
         */
        public DifferentialWheelConfig withWheelDiameter(Distance diameter){
            this.WheelCirc = Meters.of(diameter.in(Meters)*Math.PI);
            return this;
        }

        /**
         * 
         * @param side is the wheel side configuring if the motor inverted
         * @return itself
         */
        public DifferentialWheelConfig withWheelSide(WheelSide side){
            this.Inverted = side == WheelSide.RightSide;
            return this;
        }

        /**
         * 
         * @param GearRatio is the gear ratio which refers SensorToMechanismRatio
         * @return itself
         */
        public DifferentialWheelConfig withGearRatio(double GearRatio){
            this.GearRatio = GearRatio;
            return this;
        }

        /**
         * 
         * @param MaxSpeed is the max motor speed
         * @return itself
         */
        public DifferentialWheelConfig withMaxSpeed(LinearVelocity MaxSpeed){
            this.MaxSpeed = MaxSpeed;
            return this;
        }

    }

    public static class KOPChassisConfig{
        public Distance TrackWidth;
        public LinearVelocity maxVelocity;
        public Temperature TempLimit = Celsius.of(65);

        public KOPChassisConfig(){};

        public KOPChassisConfig withTrackWidth(Distance TrackWidth){
            this.TrackWidth = TrackWidth;
            return this;
        }

        public KOPChassisConfig withMaxVelocity(LinearVelocity maxVelocity){
            this.maxVelocity = maxVelocity;
            return this;
        }
    
        public KOPChassisConfig withTemperautreLimit(Temperature limit){
            this.TempLimit = limit;
            return this;
        }
    }

    public static class MecanumChassisConfig {
        public Translation2d[] WheelPlaces;
        public LinearVelocity MaxSpeed;
        public MecanumChassisConfig(){};

        /**
         * 
         * @param places configures the wheel places, setted as FL,FR,BL,BR
         * @return
         */
        public MecanumChassisConfig withWheelPlaces(Translation2d... places){
            this.WheelPlaces = places;
            return this;
        }

        public MecanumChassisConfig withMaxSpeed(LinearVelocity maxSpeed){
            this.MaxSpeed = maxSpeed;
            return this;
        }
        
    }
    
    public enum WheelSide{
        LeftSide,
        RightSide;
    }

    public interface SwerveMotor<M> {
        void setSpeed(LinearVelocity speed);
        void setSpeedPercent(double speed);
        Distance getPosition();
        LinearVelocity getVelocity();
        Temperature getTemp();
        M getMotor();
    }

    public class SwerveMotorConfig {
        public double GearRatio = 1;
        public Distance WheelCirc = Meters.of(1);
        public LinearVelocity MaxVelocity = MetersPerSecond.of(1);
        public double[] PID;

        public SwerveMotorConfig(){};

        public SwerveMotorConfig withGearRatio(double GearRatio){
            this.GearRatio = GearRatio;
            return this;
        }

        public SwerveMotorConfig withWheelDiameter(Distance diameter){
            this.WheelCirc = Meters.of(diameter.in(Meters)*Math.PI);
            return this;
        }

        public SwerveMotorConfig withMaxVelocity(LinearVelocity MaxVelocity){
            this.MaxVelocity = MaxVelocity;
            return this;
        }

        public SwerveMotorConfig withMotorPIDF(double... pid){
            if(pid.length != 4 && pid.length == 3) throw new Error("Do you lose FeedForward or motor kV value?");
            this.PID = pid;
            return this;
        }
    }

    public class SwerveModuleConfig {
        public double[] pid;
        public SwerveModuleConfig(){};

        public SwerveModuleConfig withPID(double... pid){
            this.pid = pid;
            return this;
        }
        
    }

    public class SwerveChassisConfig {
        public Translation2d[] places;
        
        public SwerveChassisConfig(){};

        public SwerveChassisConfig withSwerveModulePlaces(Translation2d... places){
            this.places = places;
            return this;
        }
    }

    public interface EncoderFrame<E>{
        Angle getPosition();
        Rotation2d getRotation2d();
        E getEncoder();
    }

    public interface GyroInterface<T> {
        Rotation2d getRotation2d();
        T getGyro();
    }

}