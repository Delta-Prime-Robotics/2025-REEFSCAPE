// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.RobotContainer;
import frc.robot.Constants.IO;
import frc.robot.Constants.IO.*;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

  
public class LEDSubsystem extends SubsystemBase {
  private static final int kLength = 128;
  private static final int kBrightness = 50;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer; 
  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(IO.kLedPort);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(m_buffer.getLength());
    m_led.start();
    setPattern(LEDPattern.solid(Color.kRed));
    // setDefaultCommand(runPattern(robotState()));
  }


  public void robotState() {
    LEDPattern base = LEDPattern.kOff;

    if (RobotState.isAutonomous()) {
        base = LEDPattern.solid(Color.kPurple)
        .synchronizedBlink(RobotController::getRSLState);
    }

    if (RobotState.isTeleop()) {
        base = LEDPattern.solid(Color.kGreen);
    //    .synchronizedBlink(RobotController::getRSLState);
    }
    
    this.setDefaultCommand(runPattern(base.atBrightness(Percent.of(kBrightness))));

    if (RobotState.isDisabled()){
      base = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(kBrightness));
      setPattern(LEDPattern.solid(Color.kRed));
    }
  }

   /** Creates a command that runs a pattern on the entire LED strip.
   * @param pattern the LED pattern to run*/
  public Command runPattern(LEDPattern pattern) {
    return runEnd(
      () -> pattern.applyTo(m_buffer),
      () -> LEDPattern.kOff.applyTo(m_buffer)
    );
  }

   /** Creates a method that runs a pattern on the entire LED strip.
   * @param pattern the LED pattern to run*/
  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(m_buffer);
  }
   
  public Command discoMode(){
    return runEnd(()-> setPattern(LEDPattern.rainbow(50, 128)),
      () -> setPattern(LEDPattern.kOff));
  }

  public Command denial() {
    return runPattern(LEDPattern.solid(Color.kOrangeRed))
    .withTimeout(2);
  }
  
  @Override
  public void periodic() {
    robotState();
    // This method will be called once per scheduler run
    m_led.setData(m_buffer);
  }
}
