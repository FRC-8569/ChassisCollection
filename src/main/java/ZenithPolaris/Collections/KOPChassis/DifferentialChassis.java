package ZenithPolaris.Collections.KOPChassis;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.studica.frc.AHRS;

import ZenithPolaris.Collections.utils.DifferentialDrive;

public class DifferentialChassis extends SubsystemBase{
    public DifferentialDrive.DifferentialWheel<?> LeftWheel, RightWheel;
    public DifferentialDrive.ChassisConfig config;
    public AHRS gyro;

    public DifferentialDriveKinematics kinematics;
    public DifferentialDriveOdometry odometry;

    public DifferentialChassis(DifferentialDrive.DifferentialWheel<?> LeftWheel, DifferentialDrive.DifferentialWheel<?> RightWheel ,DifferentialDrive.ChassisConfig config){
        this.LeftWheel = LeftWheel;
        this.RightWheel = RightWheel;

        gyro = new AHRS(config.gyroCom);
        kinematics = new DifferentialDriveKinematics(config.TrackWidth);
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), LeftWheel.getPosition(), RightWheel.getPosition());
    }

    public DifferentialDriveWheelPositions getPositions(){
        return new DifferentialDriveWheelPositions(
            LeftWheel.getPosition(), 
            RightWheel.getPosition());
    }

    public DifferentialDriveWheelSpeeds getSpeeds(){
        return new DifferentialDriveWheelSpeeds(
            LeftWheel.getVelocity(),
            RightWheel.getVelocity()
        );
    }

    public void drive(DifferentialDriveWheelSpeeds speeds){
        speeds.desaturate(config.MaxSpeed);
        LeftWheel.setVelocity(MetersPerSecond.of(speeds.leftMetersPerSecond));
        RightWheel.setVelocity(MetersPerSecond.of(speeds.rightMetersPerSecond));
    }

}
