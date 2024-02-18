package frc.robot;

import java.lang.invoke.MethodHandles;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TunePIDTalonFX extends Command {

    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    TalonFX4237 motor;
    int deviceID;

    Slot0Configs slot0Configs = new Slot0Configs();

    boolean enablePrevious;
    boolean disablePrevious;

    LinearFilter KvSmooth;
    LinearFilter egrSmooth;

    int flashDisabledDS = 0;

    TunePIDTalonFX(TunePID tunePID, int deviceID)
    {
        this.deviceID = deviceID;
        addRequirements(tunePID);
    }

  public void initialize()
  {
    motor = new TalonFX4237(deviceID, "rio", "Test Motor");
    SmartDashboard.putNumber("deviceID", deviceID);

    enablePrevious = true; // initialize to force change of state to come up in enable false disable true
    disablePrevious = false;
    KvSmooth = LinearFilter.movingAverage(50);
    egrSmooth = LinearFilter.movingAverage(50);
    SmartDashboard.putNumber("kP", 0.);
    SmartDashboard.putNumber("kI", 0.);
    SmartDashboard.putNumber("kD", 0.);
    SmartDashboard.putNumber("kS", 0.);
    SmartDashboard.putNumber("kV", 0.);
    SmartDashboard.putNumber("velocity setpoint [rps]", 0.);
    SmartDashboard.putNumber("engineering units multiplicative factor", 0);
    SmartDashboard.putNumber("velocity plot", 0); // 19 to 21
    SmartDashboard.putNumber("current velocity [rps]", 0);
    SmartDashboard.putNumber("PID closed loop error",0); // -1 to 1
    SmartDashboard.putNumber("motor voltage", 0); // 2.2
    SmartDashboard.putNumber("tentative Kv", 0);
    SmartDashboard.putNumber("tentative Kv plot", 0);
    SmartDashboard.putNumber("velocity engineering units", 0);
    SmartDashboard.putBoolean("enable controller", false);
    SmartDashboard.putBoolean("disable controller", true);

    motor.setSafetyEnabled(false);

    motor.setupFactoryDefaults();
    motor.setupInverted(true);
    motor.setupCoastMode();
    // motor.setupVelocityConversionFactor(1.0);
    motor.setupCurrentLimit(30.0, 35.0, 0.5);    
  }

  public void execute()
  {
    slot0Configs.kP = 0.0; // volts / rotation/sec
    slot0Configs.kI = 0; // volts / rotation
    slot0Configs.kD = 0.0; // volts / rotations/sec/sec
    slot0Configs.kS = 0.0; // max voltage that doesn't move the motor 
    slot0Configs.kV =  0.0; // volts / rotation per second
    // kF for other PID controllers is essentially this kV and maybe the kS crudely wrapped into it, too.
    // There is also an arbitrary feedforward that can be used if kS, kV (or others' kF) aren't adequate

    boolean enable = SmartDashboard.getBoolean("enable controller", false);
    boolean disable = SmartDashboard.getBoolean("disable controller", true);

    // manage change of state with fake radio button type logic
    if(!enable && !disable) // one has to be choosen
    {
        enable = enablePrevious;
        disable = disablePrevious;
    }

    if(enablePrevious && disable) // changing to disabled
    {
        enable = false;
        enablePrevious = false;
    }
    else
    if(disablePrevious && enable) // changing to enabled
    {
        disable = false;
        disablePrevious = false;
    }
    
    if(DriverStation.isDisabled()) // make controller disabled so it doesn't come on if robot enabled
    {
        flashDisabledDS++;
        enable = false;
        enablePrevious = false;
        disable = true;
        disablePrevious = true;
        if(flashDisabledDS < 30)
        {
            SmartDashboard.putString("DS status", "DRIVERSTATION IS DISABLED");
        }
        else
        if(flashDisabledDS < 60)
        {
            SmartDashboard.putString("DS status", "driverstation is disabled");
        }
        else
        {
            flashDisabledDS = 0;
        }
    }
    else
    {
        SmartDashboard.putString("DS status", "DriverStation is enabled");
    }

    SmartDashboard.putBoolean("enable controller", enable);
    SmartDashboard.putBoolean("disable controller", disable);

    double egrConversionFactor = 1.;

    if(enable)
    {
        enablePrevious = true;

        slot0Configs.kP = SmartDashboard.getNumber("kP", 0.); // .1
        slot0Configs.kI = SmartDashboard.getNumber("kI", 0.); // .0
        slot0Configs.kD = SmartDashboard.getNumber("kD", 0.); // .004
        slot0Configs.kS = SmartDashboard.getNumber("kS", 0.);//0.13
        slot0Configs.kV =  SmartDashboard.getNumber("kV", 0.); // .109
        double velocitySetpoint = SmartDashboard.getNumber("velocity setpoint [rps]", 0.); // 20
        egrConversionFactor = SmartDashboard.getNumber("engineering units multiplicative factor", 1.);

        // motor.getConfigurator().apply(slot0Configs);
        motor.setupPIDController(0, slot0Configs.kP, slot0Configs.kI, slot0Configs.kD, slot0Configs.kS, slot0Configs.kV);

        // final VelocityVoltage requestVelocity = new VelocityVoltage(velocitySetpoint);
        // motor.setControl(requestVelocity);
        motor.setControlVelocity(velocitySetpoint);
    }

    else
    if(disable)
    {
        disablePrevious = true;

        motor.set(0.);

        KvSmooth.reset();
        egrSmooth.reset();
        return;
    }

    else
    {
        System.out.println("undefined enable/disabled");
    }

    double
    tentativeKv = 0.;
    if (motor.getVelocity() != 0.)
    {
        // tentativeKv = (motor.getMotorVoltage().getValueAsDouble()-slot0Configs.kS)/getVelocity();
        tentativeKv = (motor.getMotorVoltage() - slot0Configs.kS) / motor.getVelocity();
    }
    tentativeKv = KvSmooth.calculate(tentativeKv);

    var
    velocityEgrUnits = egrConversionFactor*egrSmooth.calculate(motor.getVelocity());

    SmartDashboard.putNumber("velocity plot", motor.getVelocity()); // 19 to 21
    SmartDashboard.putNumber("current velocity [rps]", motor.getVelocity());
    // SmartDashboard.putNumber("PID closed loop error", motor.getClosedLoopError().getValueAsDouble()); // -1 to 1
    SmartDashboard.putNumber("motor voltage", motor.getMotorVoltage()); // 2.2
    SmartDashboard.putNumber("tentative Kv", tentativeKv);
    SmartDashboard.putNumber("tentative Kv plot", tentativeKv);
    SmartDashboard.putNumber("velocity engineering units", velocityEgrUnits);
  }

  public void end(boolean interrupted)
  {
    motor.set(0.);
  }

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  public boolean isFinished() {
    return false; // always false; let the trigger handle start/stop logic
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}

//FIXME     To enable the feature, here's the new config below.
//FIXME cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
//FIXME use kS = 0 if this option is NOT selected!!!!!!!!!!!! or only run in the + direction!!!!!!!!!!
