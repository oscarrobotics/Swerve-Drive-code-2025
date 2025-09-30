package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.fasterxml.jackson.databind.deser.impl.FailingDeserializer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import java.util.Map;
import java.util.function.BooleanSupplier;

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

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
// import frc.robot.util.Constants;
import frc.robot.util.Constants.*;

public class Elevator extends SubsystemBase{
    // All hardware classes already have WPILib integration
    final TalonFX m_elevator_motor = new TalonFX(51);
    final TalonFX m_elevator_motor_follower = new TalonFX(52);

    final CANcoder m_elevator_CANcoder = new CANcoder(59);
    

    final TalonFXSimState m_elevator_motorSim = m_elevator_motor.getSimState();

    final PositionTorqueCurrentFOC m_elevator_motorOut = new PositionTorqueCurrentFOC(0);

    final MotionMagicExpoTorqueCurrentFOC m_elevator_motorOut_mm = new MotionMagicExpoTorqueCurrentFOC(0);
    // final MotionMagicExpoTorqueCurrentFOC m_elevator_motorOut_mm = new MotionMagicExpoTorqueCurrentFOC(0);
    // used normal motion magic for to try and get it to work, but elevator was secretly mechnaically bad 
    // so probably will revert back to expo when fixed
    // final MotionMagicTorqueCurrentFOC m_elevator_motorOut_mm = new MotionMagicTorqueCurrentFOC(0);
    // final PositionTorqueCurrentFOC m_elevator_motorOut_mm = new PositionTorqueCurrentFOC(0);
    
    private final NeutralOut m_brake = new NeutralOut();




    public final Angle k_elevator_bottom = k_elevator.k_bottom; 
    public final Angle k_elevator_min_rot = k_elevator.k_min_rot;
    public final Angle k_elevator_max_rot = k_elevator.k_max_rot;

    public final Distance k_min_length =  k_elevator.k_min_length;    
    public final Distance k_max_length = k_elevator.k_max_length;




    public final Angle k_stowed =  k_elevator.k_stowed;//0.08
    public final Angle k_load =  k_elevator.k_load;//0.08

    public Angle k_coral_level_sense_postion_1 = k_elevator.k_coral_level_sense_postion_1; //trought
    public Angle k_coral_level_sense_postion_2 = k_elevator.k_coral_level_sense_postion_2; // level 2
    public Angle k_coral_level_sense_postion_3 = k_elevator.k_coral_level_sense_postion_3; // level 3
    public Angle k_coral_level_sense_postion_4 = k_elevator.k_coral_level_sense_postion_4; // level 4
    

    public final Mass k_carrage_mass = Kilogram.of(13);



    private final ElevatorSim m_elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60(2), 
        11.7/3, 
        k_carrage_mass.in(Kilograms), 
        0.025,
        k_min_length.in(Meters), 
        k_max_length.in(Meters), 
        false, 
        k_min_length.in(Meters)
        );

    private final Mechanism2d m_mech2d =
        new Mechanism2d(6, k_max_length.in(Meters));
    private final MechanismRoot2d m_mech2dRoot =
        m_mech2d.getRoot("Elevator Root", 3, 0.0);
    private final MechanismLigament2d m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90, 6, new Color8Bit(Color.kRed))
        );

    private final TalonFXConfiguration m_elevator_config = new TalonFXConfiguration();



    ///shuffleboard 
    /// 
        
    private ShuffleboardTab  m_tab = Shuffleboard.getTab("Elevator Tuning");


    private GenericEntry sh_coral_position_1 = m_tab.addPersistent("coral_position_1", k_coral_level_sense_postion_1.magnitude()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",k_elevator_min_rot.magnitude(),"max",k_elevator_max_rot.magnitude())).getEntry(); 
    private GenericEntry sh_coral_position_2 = m_tab.addPersistent("coral_position_2", k_coral_level_sense_postion_2.magnitude()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",k_elevator_min_rot.magnitude(),"max",k_elevator_max_rot.magnitude())).getEntry();
    private GenericEntry sh_coral_position_3 = m_tab.addPersistent("coral_position_3", k_coral_level_sense_postion_3.magnitude()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",k_elevator_min_rot.magnitude(),"max",k_elevator_max_rot.magnitude())).getEntry();
    private GenericEntry sh_coral_position_4 = m_tab.addPersistent("coral_position_4", k_coral_level_sense_postion_4.magnitude()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",k_elevator_min_rot.magnitude(),"max",k_elevator_max_rot.magnitude())).getEntry();

    // private GenericEntry sh_sim= m_tab.add("Elevator Sim", m_mech2d);
    // private GenericEntry sh_kp = m_tab.add("Elevator kP", k_default_kp).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",10,"max",200)).getEntry(); 
    // private GenericEntry sh_ki = m_tab.add("Elevator kI", k_default_ki).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",100)).getEntry();
    // private GenericEntry sh_kd = m_tab.add("Elevator kD", k_default_kd).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",40)).getEntry();
    // private GenericEntry sh_kg = m_tab.add("Elevator kG", k_default_kg).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",-10,"max",10)).getEntry();
    // private GenericEntry sh_kff = m_tab.add("Elevator kff", k_default_kff).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",40)).getEntry();
    // private GenericEntry sh_kff_offset = m_tab.add("Elevator kff offset", k_default_kff_offset).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",-10,"max",10)).getEntry();
    // private GenericEntry sh_current_limit = m_tab.add("Elevator Current Limit", k_current_limit).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",150)).getEntry();
    // private GenericEntry sh_cvelocity = m_tab.add("Elevator Cruise Velocity", k_default_cVelocity).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",1)).getEntry();
    // private GenericEntry sh_kv = m_tab.add("Elevator kV", k_default_kV).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",20)).getEntry();
    // private GenericEntry sh_ka = m_tab.add("Elevator kA", k_default_kA).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",20)).getEntry();


    
    // Shuffleboard.getTab("tuning").add("Elevator Sim", m_mech2d);


  

    public Elevator(){
        
        
        
        // motor configuration section
        

        
        m_elevator_config.Slot0.kS = k_elevator.k_0_ks;
        m_elevator_config.Slot0.kP = k_elevator.k_0_kp;
        m_elevator_config.Slot0.kI = k_elevator.k_0_ki;
        m_elevator_config.Slot0.kD = k_elevator.k_0_kd;
        m_elevator_config.Slot0.kG = k_elevator.k_0_kg;
        
        m_elevator_config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        // Peak output of 20 A
        m_elevator_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(k_elevator.k_current_limit)) 
        .withPeakReverseTorqueCurrent(Amps.of(-k_elevator.k_current_limit));
        CurrentLimitsConfigs elecurent = new CurrentLimitsConfigs().withStatorCurrentLimit(k_elevator.k_current_limit).withSupplyCurrentLimit(k_elevator.k_current_limit);



        //for motion magic controls
        m_elevator_config.MotionMagic.MotionMagicCruiseVelocity = k_elevator.k_0_cruiseVel;
        m_elevator_config.MotionMagic.MotionMagicAcceleration = k_elevator.k_0_Acceleration;
        m_elevator_config.MotionMagic.MotionMagicJerk = k_elevator.k_0_jerk;
        m_elevator_config.MotionMagic.MotionMagicExpo_kV = k_elevator.k_0_MM_kV;
        m_elevator_config.MotionMagic.MotionMagicExpo_kA = k_elevator.k_0_MM_kA;


        // bind the remote encoder to the mount motor

        CANcoderConfiguration m_elevator_CANcoder_config = new CANcoderConfiguration();
        m_elevator_CANcoder_config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.85));
        m_elevator_CANcoder_config.MagnetSensor.withMagnetOffset(k_elevator.k_mag_sensor_offset);
        m_elevator_CANcoder_config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
       
        
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = m_elevator_CANcoder.getConfigurator().apply(m_elevator_CANcoder_config);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
        }


        
        m_elevator_config.Feedback.FeedbackRemoteSensorID = m_elevator_CANcoder.getDeviceID();
        m_elevator_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        m_elevator_config.Feedback.SensorToMechanismRatio = k_elevator.k_sensor_to_mechanism;//1.486486
        m_elevator_config.Feedback.RotorToSensorRatio = k_elevator.k_rotor_to_sensor;//1.486486
        
        
        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status = m_elevator_motor.getConfigurator().apply(m_elevator_config);
        m_elevator_motor.getConfigurator().apply(elecurent);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
        }

        m_elevator_motor_follower.setControl(new Follower(m_elevator_motor.getDeviceID(), false));

        // m_elevator_motor.setPosition(0);
        
     

        

        SmartDashboard.putData("Update elevator positions", new InstantCommand(this::update_locations));
    //    SmartDashboard.putData(m_tab.getTitle()+"/Update Elevator PID", new InstantCommand(this::configure_from_dash));
        
        m_tab.addDoubleArray("MotionGraph", this::publish_motion_data).withWidget(BuiltInWidgets.kGraph);
        m_tab.addDoubleArray("ControlGraph", this::publish_control_data).withWidget(BuiltInWidgets.kGraph);
       
    
        

    }

    private Angle validate_and_convert(Distance claw_height){

        // Takes in a value of Distance representing the claws center axel height from the groung
        // Validates the input to make sure that the hieght is within range 
        // Converts the axel hieght to travel distance and then to the rotaition positon of the motor/sensor
        // Then validate the rotaions to makes sure the motor is not set to an out of range position incase the calculations are bugged


        return Rotations.of(10);
      


    }
    
    public boolean is_stowed(){

        

        return m_elevator_motor.getPosition().getValueAsDouble()<0.3;
        // return m_elevator_CANcoder.getAbsolutePosition().getValue().lt(Rotation.of(0.2));
    }

        
        

    
    private void set_elevator_position(Distance position){

        // gt is greater than 
        if (position.gt( k_max_length)){
         
            // logger.log(position + " requested is greater than the max position ");
            position = k_max_length;

        }
        else if (position.lt(k_min_length)){

            //logger.log(position + " requested is less than the minimum position");
            position = k_min_length;

        }

        //calucaulate the conversion from meters to rotations
        double ratio = (position.minus(k_min_length)).div(k_max_length.minus(k_min_length)).baseUnitMagnitude();
    
        Angle output = k_elevator_max_rot.minus(k_elevator_min_rot).times(ratio).plus(k_elevator_min_rot);

        output = output.gt(k_elevator_max_rot) ? output : k_elevator_max_rot; 


        m_elevator_motor.setControl(m_elevator_motorOut.withPosition(output.in(Rotations)));
        

    }


   

     


    public void set_elevator_position_mm(Angle posision){

        // // gt is greater than 
        if (posision.gt(k_elevator.k_max_rot)){
         
            // logger.log(position + " requested is greater than the max position ");
            posision = k_elevator.k_max_rot;

        }
        else if (posision.lt(k_elevator.k_min_rot)){

            //logger.log(position + " requested is less than the minimum position");
            posision = k_elevator.k_min_rot;

        }

        // //calucaulate the conversion from meters to rotations
        // double ratio = (posision.minus(k_min_Distance)).div(k_max_Distance.minus(k_min_Distance)).baseUnitMagnitude();
    
        // Angle output = k_elevator_max_rot.minus(k_elevator_min_rot).times(ratio).plus(k_elevator_min_rot);

        // output = output.gt(k_elevator_max_rot) ? output : k_elevator_max_rot; 
        double ff_factor = (posision.div(k_elevator_max_rot).magnitude())-0.5*2;

        Current ffCurrent = Amps.of(k_elevator.k_0_kff).times(ff_factor);

        // System.out.println("position set "+ posision.in(Rotation) );
        // double cV = k_elevator.k_0_cruiseVel;
        // if(m_elevator_motor.getPosition().getValue().lt(Rotation.of(0.2))){
        //     cV = k_elevator.k_0_cruiseVel/3;
        // }

        
        m_elevator_motor.setControl(m_elevator_motorOut_mm.withPosition(posision.in(Rotations)).withFeedForward(ffCurrent));
        

    }



    public BooleanSupplier at_position(double tolerance){
        
        BooleanSupplier position_trigger = ()->{

             return Math.abs(m_elevator_motor.getClosedLoopError().getValueAsDouble())<tolerance*2 &&
             Math.abs(m_elevator_motor.getVelocity().getValueAsDouble())<tolerance*2;
        }
        ;
        
        return position_trigger;
    }

    public BooleanSupplier at_position(){

        return at_position(0.005);
    }

    public double get_reference(){

        return m_elevator_motor.getClosedLoopReference().getValueAsDouble();
    }
        

    // public Command set_position_command(Distance position ){
    //     return run(()->set_elevator_position_mm(position));
        
    // }

    public Command set_position_command_angle(Angle position ){
        return runOnce(()->set_elevator_position_mm(position));
        
    }
    public void set_position_angle(Angle position ){
        runOnce(()->set_elevator_position_mm(position));
        
    }


    public Angle height2pos(Distance Height){

        var length = Height.minus(k_elevator.k_height_offset);

        Angle out_pos = Rotation.of(0.169696969);// output this in error, not too low or too high

        if(length.lt(k_elevator.k_1q_length)){

            var length_range = k_elevator.k_1q_length.minus(k_elevator.k_min_length);
            var prop_length = length.minus(k_elevator.k_min_length);
            var legth_ratio = prop_length.div(length_range);

            var pos_range = k_elevator.k_1q_length_sense.minus(k_elevator.k_min_length_sense);
            var prob_pos = legth_ratio.times(pos_range);
            var offset_pos = prob_pos.plus(k_elevator.k_min_length_sense);
            out_pos = offset_pos;

        }
        else if (length.lt(k_elevator.k_mid_length)){
            var length_range = k_elevator.k_mid_length.minus(k_elevator.k_1q_length);
            var prop_length = length.minus(k_elevator.k_1q_length);
            var legth_ratio = prop_length.div(length_range);

            var pos_range = k_elevator.k_mid_length_sense.minus(k_elevator.k_1q_length_sense);
            var prob_pos = legth_ratio.times(pos_range);
            var offset_pos = prob_pos.plus(k_elevator.k_1q_length_sense);
            out_pos = offset_pos;
        }
        else if (length.lt(k_elevator.k_3q_length)){
            var length_range = k_elevator.k_3q_length.minus(k_elevator.k_mid_length);
            var prop_length = length.minus(k_elevator.k_mid_length);
            var legth_ratio = prop_length.div(length_range);

            var pos_range = k_elevator.k_3q_length_sense.minus(k_elevator.k_mid_length_sense);
            var prob_pos = legth_ratio.times(pos_range);
            var offset_pos = prob_pos.plus(k_elevator.k_mid_length_sense);
            out_pos = offset_pos;
        }
        else if (length.lt(k_elevator.k_max_length)){
            var length_range = k_elevator.k_max_length.minus(k_elevator.k_3q_length);
            var prop_length = length.minus(k_elevator.k_3q_length);
            var legth_ratio = prop_length.div(length_range);

            var pos_range = k_elevator.k_max_length_sense.minus(k_elevator.k_3q_length_sense);
            var prop_pos = legth_ratio.times(pos_range);
            var offset_pos = prop_pos.plus(k_elevator.k_3q_length_sense);
            out_pos = offset_pos;
        }
        

        return out_pos;

        
    }
    

    


    @Override
    public void simulationPeriodic() {
        
        m_elevatorSim.setInput(m_elevator_motorSim.getMotorVoltage());

        m_elevatorSim.update(0.020);

        
        var elevatorVelocity = 
            (m_elevatorSim.getVelocityMetersPerSecond()/0.025);

        m_elevator_motorSim.setRawRotorPosition(m_elevatorSim.getPositionMeters());
        m_elevator_motorSim.setRotorVelocity(elevatorVelocity);

        m_elevatorMech2d.setLength(m_elevatorSim.getPositionMeters());
    }

    // @Override
    // public void periodic() {
    //     super.periodic();
        
    // }
    private double[] motion_data = new double[4];
    public double[] publish_motion_data(){

        // // put data important for charaterizing the data to the smart dashboard
        // double set_point = m_elevator_motor.getClosedLoopReference().getValueAsDouble();
        // double error = m_elevator_motor.getClosedLoopError().getValueAsDouble();
        // double tcurrent = m_elevator_motor.getTorqueCurrent().getValueAsDouble();
        // double velocity = m_elevator_motor.getVelocity().getValueAsDouble();
        // double acceleration = m_elevator_motor.getAcceleration().getValueAsDouble();
        // double position = m_elevator_motor.getPosition().getValueAsDouble();

        // Shuffleboard.getTab("tuning").getComponents)
        // Shuffleboard.getTab("tuning").add("Elevator Error", error).withWidget(BuiltInWidgets.kGraph);
        // Shuffleboard.getTab("tuning").add("Elevator Torque Current", tcurrent).withWidget(BuiltInWidgets.kGraph);
        // Shuffleboard.getTab("tuning").add("Elevator Velocity", velocity).withWidget(BuiltInWidgets.kGraph);
        // Shuffleboard.getTab("tuning").add("Elevator Acceleration", acceleration).withWidget(BuiltInWidgets.kGraph);
        // Shuffleboard.getTab("tuning").add("Elevator Position", position).withWidget(BuiltInWidgets.kGraph);
        // Shuffleboard.getTab("tuning").add("Elevator Sim", m_mech2d);

        motion_data[0] = m_elevator_motor.getClosedLoopReference().getValueAsDouble();
        motion_data[1] = m_elevator_motor.getPosition().getValueAsDouble();
        motion_data[2] = m_elevator_motor.getVelocity().getValueAsDouble();
        motion_data[3] = m_elevator_motor.getAcceleration().getValueAsDouble();

        return motion_data;
        
    }


    private double[] control_data = new double[3];
    
    private double[] publish_control_data(){
        
        control_data[0] = m_elevator_motor.getClosedLoopError().getValueAsDouble();
        control_data[1] = m_elevator_motor.getTorqueCurrent().getValueAsDouble();
        control_data[2] = m_elevator_motor.getClosedLoopReferenceSlope().getValueAsDouble();
        return control_data;
    }
    
    public void update_locations(){
       
        k_coral_level_sense_postion_1 = Rotation.of(sh_coral_position_1.getDouble(k_coral_level_sense_postion_1.magnitude()));
        k_coral_level_sense_postion_2 = Rotation.of(sh_coral_position_2.getDouble(k_coral_level_sense_postion_2.magnitude()));
        k_coral_level_sense_postion_3 = Rotation.of(sh_coral_position_3.getDouble(k_coral_level_sense_postion_3.magnitude()));
        k_coral_level_sense_postion_4 = Rotation.of(sh_coral_position_4.getDouble(k_coral_level_sense_postion_4.magnitude()));


        // System.out.println(k_coral_level_sense_postion_1);


    }




    public void configure_from_dash(){
        // // configure the motor from the smart dashboard
        // m_elevator_config.Slot0.kP = sh_kp.getDouble(k_default_kp); 
        // m_elevator_config.Slot0.kI = sh_ki.getDouble(k_default_ki);
        // m_elevator_config.Slot0.kD = sh_kd.getDouble(k_default_kd);
        // m_elevator_config.Slot0.kG = sh_kg.getDouble(k_default_kg);
        // k_default_kff = sh_kff.getDouble(k_default_kff);
       
        
        // m_elevator_config.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(sh_current_limit.getDouble(k_current_limit)))
        // .withPeakReverseTorqueCurrent(Amps.of(-sh_current_limit.getDouble(k_current_limit)));

        // m_elevator_config.MotionMagic.MotionMagicCruiseVelocity = sh_cvelocity.getDouble(k_default_cVelocity);
        // m_elevator_config.MotionMagic.MotionMagicExpo_kV =sh_kv.getDouble(k_default_kV);
        // m_elevator_config.MotionMagic.MotionMagicExpo_kA = sh_ka.getDouble(k_default_kA);


        // StatusCode status = StatusCode.StatusCodeNotInitialized;

        // for (int i = 0; i < 5; ++i) {
        //     status = m_elevator_motor.getConfigurator().apply(m_elevator_config);
        //     if (status.isOK()) break;
        // }
        // if (!status.isOK()) {
        //     System.out.println("Could not apply configs, error code: " + status.toString());
        // }
        
        // System.out.println("pid Updated");


    }
    

}

//k_coral_level_sense_postion_1

//Elevator heights

//scoring heights

//sensor

//READ ME IF YOUR LOOKING FOR ELEVATOR COMMANDS
//For those using ctrl + f to look for the command lines that set the height of the elevator and ect. They are in eleclaw.java, and the variables for the heights are in Elevator.java
//If this information is just wrong or becomes outdated, please change or delete it