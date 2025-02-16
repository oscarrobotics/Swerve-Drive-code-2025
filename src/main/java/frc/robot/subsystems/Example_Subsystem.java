package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Example_Subsystem extends SubsystemBase { 

    // Eample subsytem resembles a shooter on a pivot base;

    //velocity torque foc exaple code, for when you have to control the speed of the mechanism 
    //will be the motor used for the shooter wheel
    private final TalonFX m_velocityFoC = new TalonFX(0);

    private final TalonFXSimState  m_velocityFOC_sim = m_velocityFoC.getSimState();

    private final VelocityTorqueCurrentFOC m_velocityFOC_out = new VelocityTorqueCurrentFOC(0);

    // position torquecurrentFox example code, for when have to control the position of the nechanism
    // will be the motor used for the twist mount
    private final TalonFX m_postionFOC = new TalonFX(0);
    
    private final TalonFXSimState m_postionFOC_sim =  m_postionFOC.getSimState();

    private final PositionTorqueCurrentFOC m_positionFOC_out = new PositionTorqueCurrentFOC(0);

    
    // Sysid Routine code. Sysid is a gui tool for runing test inputis into the mechanism 
    // and measureing how it responds, from this data Sysid will tell you parameters used
    // in the configuration of the controler behavior so the mechanism moves correctly.
    // this is an alternative to guessing/estaimateing the parameters

    // Sysid Code For charaterizing the velocity based mechanism
    private final VelocityTorqueCurrentFOC m_positionFOC_sysid_out = new VelocityTorqueCurrentFOC(0);
    
    private final SysIdRoutine m_sysIdRoutineVelocityFOC = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )

    );

    

    public Example_Subsystem(){

    }

}
