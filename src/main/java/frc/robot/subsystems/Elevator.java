package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase{
    // All hardware classes already have WPILib integration
    final TalonFX m_talonFX = new TalonFX(0);
    final CANcoder m_cancoder = new CANcoder(0);

    final TalonFXSimState m_talonFXSim = m_talonFX.getSimState();

    final DutyCycleOut m_talonFXOut = new DutyCycleOut(0);

    final TalonFXConfiguration m_talonFXConfig = new TalonFXConfiguration();
    final CANcoderConfiguration m_cancoderConfig = new CANcoderConfiguration();

    InvertedValue m_talonFXInverted = InvertedValue.CounterClockwise_Positive;



    public Elevator(){
        
        m_talonFX.setControl(m_talonFXOut);
    }


}
