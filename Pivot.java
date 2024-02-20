package frc.robot;

import java.lang.invoke.MethodHandles;

public class Pivot extends Subsystem4237 {

    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private TalonFX4237 motor;

    Pivot(int deviceID, String canbus, String motorControllerName)
    {
        motor = new TalonFX4237(deviceID, canbus, motorControllerName);
        motor.setSafetyEnabled(false); 
        motor.setupFactoryDefaults();
        motor.setupInverted(true);
        motor.setupCoastMode();
    }
    TalonFX4237 getMotor()
    {
        return motor;
    }
}
        // motor.setupCurrentLimit(30.0, 35.0, 0.5);