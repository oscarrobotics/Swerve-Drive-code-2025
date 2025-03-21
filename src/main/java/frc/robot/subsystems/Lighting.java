package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.concurrent.Future;
import java.util.function.Function;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Lighting extends SubsystemBase{

    
    private AddressableLED led;
    
    private int ledLength = 260; 
    private AddressableLEDBuffer ledColors= new AddressableLEDBuffer(ledLength);

    private int frame = 0;
    private int frame_max = 10; 
    private int frame_counter = 0;
    private int frame_ratio = 5;

    Color primary_color = new Color(150, 0, 0);    
    Color secondary_color = new Color(0, 0, 150);

    // private Function<Void,Void> active_animation;

    private Runnable active_animation;


    public Lighting(){

        led = new AddressableLED(0);
        led.setLength(ledLength);
        led.setColorOrder(ColorOrder.kGRB);

        // create an array of int for the LED colors
        
        for (int i = 0; i < ledLength; i++) {
            // Set the first half of the LEDs to red
            if (i < ledLength / 2) {
                ledColors.setRGB(i, 150, 0, 0);
            } else { // Set the second half to blue
                ledColors.setRGB(i, 0, 0, 150);
            }
        }   


        led.setData(ledColors);
      
        led.start();
        
        
        // active_animation = this::basicAnimation;
        active_animation = ()->setColor(0, 150, 0); // set the LED color to green
        register();
        
        
    }

    public void setColor(int r, int g, int b){
         
        for (int i = 0; i < ledLength; i++) {
                ledColors.setRGB(i, r, g, b);
        }   

        led.setData(ledColors);
    }   

    public void set_primary_color(Color color){
        primary_color = color;  

    }
    public void set_secondary_color(Color color){
        secondary_color = color;  
    }
        
    public void set_primary_color(int r, int g, int b){
        
        primary_color = new Color(r, g, b);  

    }
    public void set_secondary_color(int r, int g, int b){
        secondary_color = new Color(r, g, b);
            
    }


    public void periodic(){
        frame_counter++;
        if(frame_counter > frame_ratio){
            frame_counter = 0;
            frame++;
        }   
        if(frame > frame_max){
            frame = 0;
        }
        run(active_animation);
    
    }

    public void basicAnimation(){
        // loop 2 colors around the LED strip

        // set the parameters for the animation, only needs to be set once,
        // but due to simplify implementation, it is in the loop
        frame_max = ledLength; // the animation will reset when the loop reaches the end of the strip
        frame_ratio = 5; // how many robot loops per frame change
        // primary_color = new Color(150, 0, 0); // red
        // secondary_color = new Color(0, 0, 150); // blue

        
        for (int i = 0; i < ledLength; i++) {
            // Set the first half of the LEDs to red, but increment the dividing line every frame
            if (i <= (frame+ (ledLength / 2))%ledLength) {
                ledColors.setLED(i, primary_color);
            } 
            else { // Set the second half to blue
                ledColors.setLED(i, secondary_color);
           
             }
            }
    }


        



    }







