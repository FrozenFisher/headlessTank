/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package frc.robot.commands.leds;

import frc.robot.Constants.Settings;
import frc.robot.Constants.Ports.LED;
import frc.robot.subsystems.leds.LEDController;
import frc.robot.subsystems.Tank.TankSubsystem;
import java.util.Map;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDDefaultCommand extends Command {

    private final LEDController leds;

    private final TankSubsystem tank;


    public LEDDefaultCommand() {
        leds = LEDController.getInstance();
        tank = TankSubsystem.getInstance();
        addRequirements(leds);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void execute() {
        if (!Settings.EnabledSubsystems.LED.get()) {
            return;
        }
        if (DriverStation.isDisabled()) {
            LEDPattern base = Settings.LEDs.DISABLED;
            LEDPattern pattern = base.breathe(Units.Seconds.of(2));
            leds.applyPattern(pattern);
            return;
        }
        // These probably won't actually be what we want the LEDs to be showing
        // TODO: Figure out what we want the LEDs to show
        switch (tank.getIsTurningTo()) {
            case "Left" :{ 
                Map<Double, Color> maskSteps = Map.of(0., Color.kWhite, 0.5, Color.kBlack);
                LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(0.25));
                LEDPattern base = LEDPattern.rainbow(255, 255);
                LEDPattern second = base.mask(mask);
                LEDPattern pattern = second.reversed();
                leds.applyAll(pattern);
            }
            case "Right" :{ 
                Map<Double, Color> maskSteps = Map.of(0., Color.kWhite, 0.5, Color.kBlack);
                LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(0.25));
                LEDPattern base = LEDPattern.rainbow(255, 255);
                LEDPattern pattern = base.mask(mask);
                leds.applyAll(pattern);
            }
            case "Stopped" :{ 
                leds.applyAll(Settings.LEDs.STOPPED);
            }
            case "Straight" :{ 
                leds.applyAll(Settings.LEDs.FORWARD);
            }
        }
    }
}
