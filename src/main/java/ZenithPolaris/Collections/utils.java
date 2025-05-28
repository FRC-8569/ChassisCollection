package ZenithPolaris.Collections;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;

public class utils {
    public class DifferentialDrive {
        public enum WheelDirection{
            LeftWheel,
            RightWheel;
        }
          
        public class WheelConfigraion {
            public Distance WheelSize = Meters.of(1);
            public boolean inverted = false;
            public double GearRatio = 1.0;
            public LinearVelocity MaxSpeed = MetersPerSecond.of(1.0);
    

            public WheelConfigraion(){}
    
            /**
             * 
             * @param WheelSize is the wheel size in {@code Distance}
             * @return itself
             */
            public WheelConfigraion withWheelSize(Distance WheelSize){
                this.WheelSize = WheelSize;
                return this;
            }
    
            /**
             * 
             * @param WheelSizeInches is the wheel size in inches and it will convert into {@code Distance} while logging to the variable
             * @return  itself
             */
            public WheelConfigraion withWheelSizeInches(double WheelSizeInches){
                this.WheelSize = Inches.of(WheelSizeInches);
                return this;
            }
    
            /**
             * 
             * @param GearRatio is the gear ratio of your mechanism
             * @return itself
             */
            public WheelConfigraion withGearRatio(double GearRatio){
                this.GearRatio = GearRatio;
                return this;
            }

            /**
             * 
             * @param speed is the max speed of your robot
             * @return itself
             */
            public WheelConfigraion withMaxSpeed(LinearVelocity speed){
                this.MaxSpeed = speed;
                return this;
            }
    
            /**
             * 
             * @param direction is the Wheel Direction which inputs with {@link DifferentialDrive.WheelDirection}
             * @return
             */
            public WheelConfigraion withWheelDirection(WheelDirection direction){
                this.inverted = direction == WheelDirection.RightWheel;
                return this;
            }
        }
    
        public interface DifferentialWheel<MotorType>{
            /**
             * 
             * @param speed is the linear velocity which usually configured as {@code MetersPerSecond.of(SpeedMPS)}
             */
            void setVelocity(LinearVelocity speed);

            LinearVelocity getVelocity();
            Distance getPosition();
            Temperature getTemp();

            /**
             * 
             * @return the motor itself
             */
            MotorType getMotor();
        }    
    
        public class ChassisConfig {
            public Distance TrackWidth;
            public NavXComType gyroCom;
            public LinearVelocity MaxSpeed;

            public ChassisConfig(){};
            /**
             * 
             * @param width is the wheel center distance of the robot
             * @return itself
             */
            public ChassisConfig withTrackWidth(Distance width){
                this.TrackWidth = width;
                return this;
            }

            /**
             * 
             * @param type is the communication port of the NavX gyroscope
             * @return itself
             */
            public ChassisConfig withGyroPort(NavXComType type){
                this.gyroCom = type;
                return this;
            }


            /**
             * 
             * @param speed is the maxmimum speed of yor robot
             * @return itself
             */
            public ChassisConfig withMaxVelocity(LinearVelocity speed){
                this.MaxSpeed = speed;
                return this;
            }
        }
    }
}
