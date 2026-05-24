/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package frc.robot.subsystems.leds;

import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;
import frc.robot.Constants.Ports.LED;
import frc.robot.subsystems.Tank.TankSubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {

    private static LEDController instance;
    
        private final LEDPattern defaultPattern = Settings.LEDs.DISABLED;
    
        private AddressableLED led;
    
        private AddressableLEDBuffer buffer;
    
    
    
        public static LEDController getInstance() {
                return instance == null ? instance = new LEDController() : instance;
    }

    private LEDController() {
        this.led = new AddressableLED(Ports.LED.LED_PWM_PORT);
        this.buffer = new AddressableLEDBuffer(Settings.LED.LED_LENGTH);
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
        
        applyAll(defaultPattern);
        SmartDashboard.putData(instance);
    }



    public void applyAll(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }

    @Override
    public void periodic() {
        // NOTE: Settings.EnabledSubsystems is not defined in Constants.java.
        // Always update the LED hardware from the current buffer so default/command patterns work.
        led.setData(buffer);
    }
}
