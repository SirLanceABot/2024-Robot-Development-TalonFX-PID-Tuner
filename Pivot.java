package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Command;

public class Pivot extends Subsystem4237 {

    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private TalonFX4237 motor;
    private TunePID tunePID;

    Pivot(int deviceID, String canbus, String motorControllerName, TunePID tunePID)
    {
        this.tunePID = tunePID;
        motor = new TalonFX4237(deviceID, canbus, motorControllerName);
        motor.setSafetyEnabled(false); 
        motor.setupFactoryDefaults();
        motor.setupInverted(true);
        motor.setupCoastMode();
        MotorController4237.registerPIDTuning("Pivot", tunePID());
    }

    Command tunePID()
    {
        return new TunePIDTalonFX(tunePID, this, motor);
    }
}
        // motor.setupCurrentLimit(30.0, 35.0, 0.5);