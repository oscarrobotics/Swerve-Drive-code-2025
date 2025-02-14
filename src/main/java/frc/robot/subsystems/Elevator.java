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


public class Elevator extends SubsystemBase{
    // All hardware classes already have WPILib integration
    final TalonFX m_elevator_motor = new TalonFX(21);
    

    final TalonFXSimState m_elevator_motorSim = m_elevator_motor.getSimState();

    final PositionVoltage m_elevator_motorOut = new PositionVoltage(0);

    final TalonFXConfiguration m_elevator_motorConfig = new TalonFXConfiguration();
    
    



    public Elevator(){
        // m_elevator_motorConfig.Slot0(
        //     //PID
        // );
        
        
        // motor configuration section
        m_elevator_motor.getConfigurator().apply(m_elevator_motorConfig);


    }
    



    private void set_elevator_position(Double posision){
      
           
        m_elevator_motor.setControl(m_elevator_motorOut.withPosition(posision));
        }
        
        

    public Command get_posiCommand(Double position ){
        return run(()->set_elevator_position(position));
        
    }

}

