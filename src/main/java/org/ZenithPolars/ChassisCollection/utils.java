package org.ZenithPolars.ChassisCollection;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class utils {
    public class DifferentialDrives {
        public enum WheelSide{
            LeftSide,
            RightSide;
        }

        public class KOPWheelConfig {
            public Distance WheelCirc = Meters.of(1);
            public double GearRatio = 1;
            public WheelSide side;
            public LinearVelocity maxSpeed;

            public KOPWheelConfig(){}

            /**
             * 
             * @param diameter is the wheel diameter usually measured in inches
             * @return itself
             */
            public KOPWheelConfig withWheelDiameter(Distance diameter){
                this.WheelCirc = diameter.times(Math.PI);
                return this;
            }

            /**
             * 
             * @param GearRatio is the motor gear ratio which > 1 is speed reduction
             * @return iteself
             */
            public KOPWheelConfig withGearRatio(double GearRatio){
                this.GearRatio = GearRatio;
                return this;
            }
            
            /**
             * 
             * @param side is the wheel side of your chassis
             * @return itself
             */
            public KOPWheelConfig withWheelSide(WheelSide side){
                this.side = side;
                return this;
            }

            /**
             * 
             * @param MaxSpeed is the maxmimum motor speed of your robot, which can calculated as {@code MaxMotorSpeedRPS/GearRatio*WheelCircMeters} and it will calculate in mps
             * @return
             */
            public KOPWheelConfig withMaxSpeed(LinearVelocity MaxSpeed){
                this.maxSpeed = MaxSpeed;
                return this;
            }
        }
        

        public class KOPChassisConfig{
            public Distance TrackWidth;
            public LinearVelocity MaxSpeed;


            public KOPChassisConfig(){};
            /**
             * 
             * @param TrackWidth is the wheel center distance
             * @return itself
             */
            public KOPChassisConfig withTrackWidth(Distance TrackWidth){
                this.TrackWidth = TrackWidth;
                return this;
            }

            /**
             * 
             * @param maxSpeed is the maxmimum speed of your robot
             * @return itself
             */
            public KOPChassisConfig withMaxSpeed(LinearVelocity maxSpeed){
                this.MaxSpeed = maxSpeed;
                return this;
            }
        }
    
        public class MecanumChassisConfig {
            public Translation2d[] WheelPlaces;
            public LinearVelocity MaxVelocity;
            public AngularVelocity MaxOmega;

            public MecanumChassisConfig(){}

            /**
             * 
             * @param places is the wheel places used as {@code Translation2d}
             * @return itself
             */
            public MecanumChassisConfig withWheelPlaces(Translation2d... places){
                this.WheelPlaces = places;
                return this;
            }

            public MecanumChassisConfig withMaxVelocity(LinearVelocity MaxSpeed){
                this.MaxVelocity = MaxSpeed;
                return this;
            }

            public MecanumChassisConfig withMaxOmega(AngularVelocity MaxOmega){
                this.MaxOmega = MaxOmega;
                return this;
            }
            
        }

        public interface NormalDriveWheels<M> {
            /**
             * @param speed is the velocity will be set to the motor
             */
            void setSpeed(LinearVelocity speed);

            /**
             * @param percent is the motor output percent
             */
            void setSpeed(double percent);

            void setSpeedMPS(double speedMPS);

            /**
             * @return the motor velocity in linearvelocity, default in mps
             */
            LinearVelocity getVelocity();

            /**
             * @return the motor position in distance, default in meters
             */
            Distance getPosition();

            /**
             * @return the motor itself
             */
            M getMotor();
            
        }
    
        
        public class Swerve {

            public enum MotorUsage {
                Drive,
                Steer;
            }

            public class SwerveModuleConfig {
                public Distance WheelCirc;
                public double GearRatio;
                public double[] DrivePID, SteerPID;

                public SwerveModuleConfig(){};

                /**
                 * 
                 * @param diameter is the wheel diameter which can input like {@code config.withWheelDiameter(Inches.of(4))}
                 * @return itself
                 */
                public SwerveModuleConfig withWheelDiameter(Distance diameter){
                    this.WheelCirc = diameter.times(Math.PI);
                    return this;
                }

                /**
                 * 
                 * @param GearRatio is the gear ratio which greater than 1 reduces the speed
                 * @return itself
                 */
                public SwerveModuleConfig withGearRatio(double GearRatio){
                    this.GearRatio = GearRatio;
                    return this;
                }

                public SwerveModuleConfig withDrivePIDF(double... pidf){
                    this.DrivePID = pidf;
                    return this;
                }

                public SwerveModuleConfig withSteerPID(double... pid){
                    this.SteerPID = pid;
                    return this;
                }
            }

            public class SwerveChassisConfig {
                public Translation2d[] WheelPlaces;

                public SwerveChassisConfig(){};

                public SwerveChassisConfig withWheelPlaces(Translation2d... places){
                    this.WheelPlaces = places;
                    return this;
                }
                
            }
        
            public interface SwerveWheels<M> {
                void setSpeed(double speedMPS);
                void setSpeedPerc(double percent);
                LinearVelocity getVelocity();
                Distance getPosition();
                M getMotor();
            }
        }

        public interface Gyroscope<g> {
            Rotation2d getRotation2d();
            g getGyro();
            
        }

        public interface Encoder<e> {
            Angle getAngle();
            e getEncoder();
            
        }
    }
}