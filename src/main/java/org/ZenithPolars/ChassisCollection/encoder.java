package org.ZenithPolars.ChassisCollection;

import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.Encoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;

public class encoder {
    public class CTRECANCoder implements Encoder<CANcoder>{
        public CANcoder encoder;
        public CANcoderConfiguration encoderConfig;

        public CTRECANCoder(int CANCoderID, double Offset){
            encoder = new CANcoder(CANCoderID);
            encoderConfig = new CANcoderConfiguration();

            encoderConfig.MagnetSensor
                .withMagnetOffset(Offset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

            encoder.getConfigurator().apply(encoderConfig);
        }

        @Override
        public Angle getAngle(){
            return encoder.getAbsolutePosition().getValue();
        }

        @Override
        public CANcoder getEncoder(){
            return encoder;
        }
        
    }
}
