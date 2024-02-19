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
    String slot0ConfigsPrevious;

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
    var
    canbus = "rio";
    var
    motorControllerName = "Test Motor";
    
    motor = new TalonFX4237(deviceID, canbus, motorControllerName);
    SmartDashboard.putNumber("deviceID", deviceID);
    SmartDashboard.putString("CAN bus", canbus);
    SmartDashboard.putString("motor controller name", motorControllerName);

    enablePrevious = true; // initialize to force change of state to come up in enable false disable true
    disablePrevious = false;
    KvSmooth = LinearFilter.movingAverage(50);
    egrSmooth = LinearFilter.movingAverage(50);

    SmartDashboard.putBoolean("enable controller", false);
    SmartDashboard.putBoolean("disable controller", true);

    motor.setSafetyEnabled(false);

    motor.setupFactoryDefaults();
    motor.setupInverted(true);
    motor.setupCoastMode();
    motor.setupVelocityConversionFactor(4.0);
    motor.setupCurrentLimit(30.0, 35.0, 0.5);
    slot0ConfigsPrevious = slot0Configs.serialize();
    clearGainsSetpoints();
  }

  public void execute()
  {
    boolean enable = SmartDashboard.getBoolean("enable controller", false);
    boolean disable = SmartDashboard.getBoolean("disable controller", true);

    // manage change of enable/disable state with fake radio button type logic
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
//////////////////////////////
    // manage change of velocity/position state with almost radio button type logic
    // WPILib chooser failed and this circumvents that usage
    // both off or both on isn't appropriate so all off
    // force user to be purposeful here - it's a dangerous setting
    boolean velocity = SmartDashboard.getBoolean("velocity", false);
    boolean position = SmartDashboard.getBoolean("position", false);
    if(velocity == position)
    {
        clearGainsSetpoints();
        velocity = false;
        position = false;
        enable = false;
        enablePrevious = false;
        disable = true;
        disablePrevious = true;
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

        // kF for other PID controllers is essentially this kV and maybe the kS crudely wrapped into it, too.
        // There is also an arbitrary feedforward that can be used if kS, kV (or others' kF) aren't adequate
        slot0Configs.kP = SmartDashboard.getNumber("kP", 0.); // volts / rotation/sec
        slot0Configs.kI = SmartDashboard.getNumber("kI", 0.); // volts / rotation
        slot0Configs.kD = SmartDashboard.getNumber("kD", 0.); // volts / rotations/sec/sec
        slot0Configs.kS = SmartDashboard.getNumber("kS (Rev: 0)", 0.); // max voltage that doesn't move the motor
        slot0Configs.kV =  SmartDashboard.getNumber("kV (Rev: kF)", 0.); // volts / rotation per second
        double velocitySetpoint = SmartDashboard.getNumber("velocity setpoint", 0.);
        double positionSetpoint = SmartDashboard.getNumber("position setpoint", 0.);
        egrConversionFactor = SmartDashboard.getNumber("engineering units multiplicative factor", 1.);
        var
        slot0ConfigsNew = slot0Configs.serialize();
        if(!slot0ConfigsNew.equals(slot0ConfigsPrevious))
        {
            // motor.getConfigurator().apply(slot0Configs);
            motor.setupPIDController(0, slot0Configs.kP, slot0Configs.kI, slot0Configs.kD, slot0Configs.kS, slot0Configs.kV);
            slot0ConfigsPrevious = slot0ConfigsNew;
        }

        if ( velocity )
        {
            motor.setControlVelocity(velocitySetpoint);
        }

        else
        if ( position)
        {
            motor.setControlPosition(positionSetpoint);  
        }

        else
        {
            System.out.println("undefined tuning mode");
            return;
        }
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
        return;
    }

    double
    tentativeKv = 0.;
    if (velocity && motor.getVelocity() != 0.)
    {
        // tentativeKv = (motor.getMotorVoltage().getValueAsDouble()-slot0Configs.kS)/getVelocity();
        tentativeKv = (motor.getMotorVoltage() - slot0Configs.kS) / motor.getVelocity();
    }
    tentativeKv = KvSmooth.calculate(tentativeKv);

    double responseEgrUnits;

    if ( velocity )
    {
        responseEgrUnits = egrConversionFactor*egrSmooth.calculate(motor.getVelocity());
        SmartDashboard.putNumber("response engineering units", responseEgrUnits);
        SmartDashboard.putNumber("response plot", motor.getVelocity()); // 19 to 21
    }
    else
    {
        responseEgrUnits = egrConversionFactor*egrSmooth.calculate(motor.getPosition());
        SmartDashboard.putNumber("response engineering units", responseEgrUnits);
        SmartDashboard.putNumber("response plot", motor.getPosition());
    }
    SmartDashboard.putNumber("current velocity", motor.getVelocity());
    SmartDashboard.putNumber("current position", motor.getPosition());
    // SmartDashboard.putNumber("PID closed loop error", motor.getClosedLoopError().getValueAsDouble()); // -1 to 1
    SmartDashboard.putNumber("motor voltage", motor.getMotorVoltage()); // 2.2
    SmartDashboard.putNumber("tentative Kv", tentativeKv);
    SmartDashboard.putNumber("tentative Kv plot", tentativeKv);
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

  private void clearGainsSetpoints()
  {
    SmartDashboard.putNumber("kP", 0.);
    SmartDashboard.putNumber("kI", 0.);
    SmartDashboard.putNumber("kD", 0.);
    SmartDashboard.putNumber("kS (Rev: 0)", 0.);
    SmartDashboard.putNumber("kV (Rev: kF)", 0.);
    SmartDashboard.putNumber("velocity setpoint", 0.);
    SmartDashboard.putNumber("position setpoint", 0);
    SmartDashboard.putNumber("engineering units multiplicative factor", 0);
    SmartDashboard.putNumber("response plot", 0); // 19 to 21
    SmartDashboard.putNumber("current velocity", 0);
    SmartDashboard.putNumber("PID closed loop error",0); // -1 to 1
    SmartDashboard.putNumber("motor voltage", 0); // 2.2
    SmartDashboard.putNumber("tentative Kv", 0);
    SmartDashboard.putNumber("tentative Kv plot", 0);
    SmartDashboard.putNumber("response engineering units", 0);
    SmartDashboard.putBoolean("velocity", false);
    SmartDashboard.putBoolean("position", false);
    SmartDashboard.updateValues();
  }

}

//FIXME     To enable the feature, here's the new config below.
//FIXME cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
//FIXME use kS = 0 if this option is NOT selected!!!!!!!!!!!! or only run in the + direction!!!!!!!!!!

// final VelocityVoltage requestVelocity = new VelocityVoltage(velocitySetpoint);
// motor.setControl(requestVelocity);

/*
In a Position closed loop, the gains should be configured as follows:
• Ks - unused, as there is no target velocity
• Kv - unused, as there is no target velocity
• Ka - unused, as there is no target acceleration
• Kp - output per unit of error in position (output/rotation)
• Ki - output per unit of integrated error in position (output/(rotation*s))
• Kd - output per unit of error derivative in position (output/rps)

In a Velocity closed loop, the gains should be configured as follows:
• Ks - output to overcome static friction (output)
• Kv - output per unit of requested velocity (output/rps)
• Ka - unused, as there is no target acceleration
• Kp - output per unit of error in velocity (output/rps)
• Ki - output per unit of integrated error in velocity (output/rotation)
• Kd - output per unit of error derivative in velocity (output/(rps/s))
*/