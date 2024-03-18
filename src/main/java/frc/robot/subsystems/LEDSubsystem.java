// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  AddressableLED m_led = new AddressableLED(1);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(151);

  public enum Colours {
    WHITE,
    RED,
    BLUE
  }
  Colours defaultColour;


  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {

    

    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      defaultColour = Colours.BLUE;
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
      defaultColour = Colours.RED;
    }
    setLEDColour(defaultColour);
  }

  public void setLEDColour(Colours colour){
    for(var i = 0; i < m_ledBuffer.getLength(); i++){
      switch(colour){
        case BLUE -> m_ledBuffer.setRGB(i, 0, 255/3, 255); // blue
        case RED -> m_ledBuffer.setRGB(i, 255, 255/3, 0); // red
        case WHITE -> m_ledBuffer.setRGB(i, 255, 255, 255); // white (? needs to be tested)
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public Colours getDefaultColour(){
    return defaultColour;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
