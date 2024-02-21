package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  TunePID tunePID = new TunePID();

  TunePIDTalonFX tunePIDFlywheel;
  TunePIDTalonFX tunePIDIndex;
  TunePIDTalonFX tunePIDPivot;
  // public static Map<String, Command> tuneThese = new HashMap<>(10); // other Classes' motors register here

  @Override
  public void robotInit()
  {
    // new Flywheel(12, "rio", "Flywheel Motor", tunePID); // testing; real ones below
    new Flywheel(51, "CANivore", "Flywheel Motor", tunePID);
    new Index(52, "CANivore", "Index Motor", tunePID);
    new Pivot(53, "CANivore", "Pivot Motor", tunePID);
    MotorController4237.createPIDDashBoard();
  }

  @Override
  public void robotPeriodic()
  {
      CommandScheduler.getInstance().run();
  }
}

/*
https://www.splatco.com/skb/2665.htm

PID theory: Backlash, stickiness and other nasties
Especially in mechanical processes there are varying mechanical imperfections that can impair the function of a control system. These include things like:

Backlash, sometimes also called hysteresis. This is what you get when a normal gear or chain drive reverses. There is a small amount of slack in the coupling, so the shaft has to travel back a small amount before it has taken up the slack. Backlash will limit the attainable accuracy of the system and very likely also lead to small oscillations around the setpoint that can't be eliminated by PID adjustments. By far the recommended approach to dealing with backlash, as well as all other mechanical imperfections, is to eliminate it as far as possible. For example, low backlash gears can be used. Another trick that is used sometimes is to deliberately create a deadband in the controller. A deadband is created by making the controller ignore process error less than a certain amount. However, this can create problems of its own with certain types of system.

Stiction (sticky friction). This is the effect whereby it takes more force to start two surfaces sliding against each other than to maintain the movement. It can produce a jerky output. It is most commonly encountered in proportional control valves. Search online for stiction. Just about any sort of closed loop control will suffer if it has to deal with a significant amount of stiction, reducing the effect is the recommended approach.

One thing to be cautious about with systems that display these mechanical imperfections is that they can make unstable systems look stable when things are steady. Everything may work fine until a big disturbance comes along, and all of a sudden your device is swinging wildly from one extremity to the other, and the only way to stop it is to turn it off. You would definitely not want to sell a large number of items with this characteristic!
*/

// https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html
// elevator and arm Kg feedforward

// https://api.ctr-electronics.com/phoenix6/release/java/

// position voltage feedforward
// https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/PositionVoltage.html#FeedForward


// CTRE new feedback for Ks signed

// slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
// cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
/*
We cut a dev release that has the final kS feature addition you requested, and a fix to MotionMagicExpo getters (like closed loop reference).
Once we finish our formal test plan, it will be the next public release.
I figured you might want to confirm your result with kS first.

To enable the feature, here's the new config below.
cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

The vendordep is attached to use the tentativeÂ release.
Also the tentativeÂ CRFs are attached (vC version is for Kraken, the other for Falcon).

I confirmed kS now supports both strategies based on your config choice, default is to use profile velocity (which is how it works in the currently public release).
I tested both Kraken and Falcon.
*/
