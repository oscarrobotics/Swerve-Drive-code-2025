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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

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

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase{
    // All hardware classes already have WPILib integration
    final TalonFX m_elevator_motor = new TalonFX(21);
    

    final TalonFXSimState m_elevator_motorSim = m_elevator_motor.getSimState();

    final PositionVoltage m_elevator_motorOut = new PositionVoltage(0);

    final TalonFXConfiguration m_elevator_motorConfig = new TalonFXConfiguration();
    public final Distance k_min_dDistance =  Meters.of(0);
    public final Distance k_max_Distance = Meter.of(90);
    
    public final Distance k_coral_level_one = Meters.of(0);
    public final Distance k_coral_level_two = Meters.of(0);
    public final Distance k_coral_level_three = Meters.of(0);
    public final Distance k_coral_level_four = Meters.of(0);

    public Elevator(){
        // m_elevator_motorConfig.Slot0(
        //     //PID
        // );
        
        
        // motor configuration section
        m_elevator_motor.getConfigurator().apply(m_elevator_motorConfig);


    }
    

        
        

    
    private void set_elevator_position(Distance posision){

        // gt is greater than 
     if (posision.gt( k_max_Distance)){
         
         // logger.log(position + " requested is greater than the max position ");
         posision = k_max_Distance;

     }
     else if (posision.lt(k_min_dDistance)){

         //logger.log(position + " requested is less than the minimum position");
         posision = k_min_dDistance;

     }

     m_elevator_motor.setControl(m_elevator_motorOut.withPosition(posision.in(Meter)));
        

 }

 public Command get_posiCommand(Distance position ){
        return run(()->set_elevator_position(position));
        
    }
}

