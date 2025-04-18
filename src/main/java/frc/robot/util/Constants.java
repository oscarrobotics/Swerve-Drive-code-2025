package frc.robot.util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.*;


public class Constants {

    public static final Distance k_bellypan_height = Meters.of(0.1);// height of the top of the belly pan from the ground

    //Heights of the reef scoring rods.

    



    public static class k_elevator {

        public static final Angle k_mag_sensor_offset = Rotations.of(-0.278076);
        public static final double k_sensor_to_mechanism =1/1.486486; // sensor rotations to mechanism travel ratio
        public static final double k_rotor_to_sensor = 11.71*5.5;


        public static final Angle k_min_rot = Rotations.of(0);
        public static final Angle k_max_rot = Rotations.of(0.98);


        // map points to convert from rotations to lengths of the elevator,
        // these should stay constant if the elevator is not modified
        // can be used as verification points to ensure the elevator is working correctly
        // if verification fails, then the elevator should come back into spec if re-mesured,
        // whithout need to change the scoring goal heights
        public static final Distance k_height_offset = Meters.of(0.075);// height of top of belly pan

        public static final Distance k_min_length =  Meters.of(0);// distace from the axel to the to of the belly pan a 0
        public static final Distance k_max_length = Meter.of(1.83); // distance from the axel to belly pan at max height
        
        public static final Distance k_mid_length = Meters.of(0.91); // distance from the axel to belly pan at mid height   

        public static final Distance k_1q_length = Meters.of(0.45); // distance from the axel to belly pan at 1st quarter height
        public static final Distance k_3q_length = Meters.of(1.37); // distance from the axel to belly pan at 3rd quarter height



        public static final Angle k_min_length_sense = Rotations.of(0);
        public static final Angle k_max_length_sense = Rotations.of(0.98);

        public static final Angle k_mid_length_sense = Rotations.of(0.49);

        public static final Angle k_1q_length_sense = Rotations.of(0.24);
        public static final Angle k_3q_length_sense = Rotations.of(0.74);




        //elevator goal sensor positions, legacy fall back

        public static final Angle k_stowed =  Rotation.of(0.01);//0.08
        public static final Angle k_load =  Rotation.of(0.25);//0.08

        public static Angle k_coral_level_sense_postion_1 = Rotations.of(0.24); //trought
        public static Angle k_coral_level_sense_postion_2 = Rotations.of(0.28); // level 2
        public static Angle k_coral_level_sense_postion_3 = Rotations.of(0.46); // level 3
        public static Angle k_coral_level_sense_postion_4 = Rotations.of(0.93); // level 4


        // elevator goal leghts




        // physical Characteristics of robot/ verifcation only
        public static final Distance min_axel_height = Inches.of(12);
        public static final Distance max_axel_height = Inches.of(78);

        public static final Mass k_carrage_mass = Kilogram.of(13);



        // elevator motor constants

        public static final double k_current_limit = 70;
        // Slot 0 elevator pid gains
        public static final double k_0_ks = 0; // output to overcome static friction
        public static final double k_0_kp = 70; // proportional
        public static final double k_0_ki = 0; // integral
        public static final double k_0_kV = 3; //current/torque based control does not require velocity feed forward
        public static final double k_0_kA = 25; 
        public static final double k_0_kd = 30; //derivative
        public static final double k_0_kg = 22; //gravity; minimum ampage(?) for movement to account for opposing forces
        public static double k_0_kff = 8;
        public static double k_0_kff_offset = 0;
        // public static final double k_0_MM_kV = Volts.per(Rotations.per(Second)).ofNative(0.12*k_sensor_to_mechanism);
        public static final double k_0_MM_kV = 0.8; // voltage required to maintain a given velocity, in V/rps
        public static final double k_0_MM_kA = 0.4; // voltage required to apply a given acceleration, in V/(rps/s)
        public static final double k_0_cruiseVel = 0.6; // used for both mm and mm_expo
        
        // slot 0 "normal" motion magic gains
        public static final double k_0_Acceleration =10; //noma
        public static final double k_0_jerk = 10;

        


        
    }

    public static class k_claw{

        public static final AngularVelocity k_max_wheel_speed = RevolutionsPerSecond.of(1000/60.0);
    

    
        public static  final AngularVelocity k_max_arm_speed = RevolutionsPerSecond.of(5/60.0);



        // physical dimentions of the robot
        public final static Distance k_intake_length = Inches.of(12);
        public final static Distance k_algehook_center_length = Inches.of(20);

        //Mount/Claw wrist angles
        public  final static Angle k_min_angle =  Rotation.of(-.12);
        public final static Angle k_max_angle = Rotation.of(0.36);

        public final static Angle k_load_coral_position = Degrees.of(0);

        public final static Angle k_stowed = Rotation.of(0.35);
        public final static Angle k_alge_stowed = Rotation.of(0.10);   
        
        public final static Angle k_load = Rotation.of(0.24);

        public final static Angle k_coral_position_1 = Rotation.of(0.29);
        public final static Angle k_coral_position_mid = Rotation.of(0.18);//middle scoreing heights
        public final static Angle k_coral_position_shoot_alge = Rotation.of(0.20);//middle scoreing heights
        public final static Angle k_coral_position_get_alge = Rotation.of(0.05);//middle scoreing heights
        public final static Angle k_coral_position_high = Rotation.of(0.16);//top scoreing height OG is 0.04
    
        public final static Angle k_coral_position_floor = Rotation.of(-0.07);

        public final Angle k_alge_position_1 = Rotation.of(0);
        public final Angle k_alge_position_2 = Rotation.of(0);
        public final Angle k_alge_position_3 = Rotation.of(0);
        public final Angle k_alge_position_4 = Rotation.of(0);

        public final Angle k_process_alge_position = Rotation.of(0);
        public final Angle k_barge_alge_position = Rotation.of(0);


    }

    public static class k_intake{

    }

    public static class k_climber{
        
        public static final Angle k_max_travel = Rotations.of(5); // todo
        public static final Angle k_deploy_position = Rotations.of(	22.914062 ); // todo

        public static final double deploy_current_limit = 20;  
        public static final double climb_current_limit = 150;

        public static final AngularVelocity deploy_cruse_velocity = RPM.of(100); 
        public static final AngularVelocity climb_cruse_velocity = RPM.of(100);

        public static final double acceleration = 1;
        public static final double jerk = 10;
        public static final double kV = 10;
        public static final double kA = 6;


        // PID gains for deploy velocity
        public static final double k_deploy_velocity_ks = 1;
        public static final double k_deploy_velocity_kp = 6;
        public static final double k_deploy_kV = 12;
        public static final double k_deploy_velocity_ki = 2;
        public static final double k_deploy_velocity_kd = 0.003;
        // // PID gains for climbing velocity
        // public static final double k_climb_velocity_ks = 1;
        // public static final double k_climb_velocity_kp = 1;
        // public static final double k_climb_velocity_ki = 0;
        // public static final double k_climb_velocity_kd = 0.05;

        // PID gains for deploy position
        public static final double k_deploy_position_kV = 5 ; 
        public static final double k_deploy_position_kp = 4;
        public static final double k_deploy_position_ki = 0;
        public static final double k_deploy_position_kd = 0;
        public static final double k_deploy_position_kg = 0;
        // PID gains for climbing position
        public static final double k_climb_position_kp = 4;
        public static final double k_climb_position_ki = 0;
        public static final double k_climb_position_kd = 1;
        public static final double k_climb_position_kg = 3;




    }

    public static class k_vision{

       

        public static final String k_orange_camera_name = "orange";
        public static final Transform3d k_orange_camera_offset = new Transform3d(
            new Translation3d(Inches.of(0.25), Inches.of(13), Inches.of(32.87)), 
            new Rotation3d(Degree.of(0), Degree.of(20), Degree.of(0))    
            );

        public static final String k_green_camera_name = "green";
        public static final Transform3d k_green_camera_offset =  new Transform3d(
            new Translation3d(Inches.of(0.25), Inches.of(13), Inches.of(32.87)), 
            new Rotation3d(Degree.of(0), Degree.of(-20), Degree.of(0))
            );
        

    }

    public static class k_led{

    }

    public static class reef{

        public static final Distance k_coral_level_1 = Meters.of(0.48);
        public static final Distance k_coral_level_2 = Meters.of(0.6);
        public static final Distance k_coral_level_3 = Meters.of(1.21);
        public static final Distance k_coral_level_4 = Meters.of(1.83);

        public static final Distance k_alge_level_1 = Meters.of(0);
        public static final Distance k_alge_level_2 = Meters.of(0);
        public static final Distance k_alge_stacked = Meters.of( 0.4);
        

        public static final Distance k_barge_height = Meters.of(0);

        public static final Distance k_processor_height = Meters.of(0);


    }


    
}
