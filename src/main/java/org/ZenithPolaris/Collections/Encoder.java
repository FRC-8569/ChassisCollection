package org.ZenithPolaris.Collections;

import static edu.wpi.first.units.Units.Degrees;

import org.ZenithPolaris.Collections.utils.EncoderFrame;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class Encoder {
    public static class CTRECANCoder implements EncoderFrame<CANcoder>{
        public CANcoder encoder;
        private CANcoderConfiguration configuration;

        public CTRECANCoder(int CANCoderID, double offset){
            encoder = new CANcoder(CANCoderID);

            configuration = new CANcoderConfiguration();

            configuration.MagnetSensor
                .withMagnetOffset(offset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

            encoder.getConfigurator().apply(configuration);
        }

        @Override
        public Angle getPosition(){
            return encoder.getAbsolutePosition().getValue();
        }

        @Override
        public Rotation2d getRotation2d(){
            return Rotation2d.fromDegrees(getPosition().in(Degrees));
        }

        @Override
        public CANcoder getEncoder(){
            return encoder;
        }
    }
}
