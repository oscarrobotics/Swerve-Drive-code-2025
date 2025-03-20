package frc.robot.util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.*;


public class Constants {

    public static final Distance k_bellypan_height = Meters.of(0.1);// height of the top of the belly pan from the ground

    //Heights of the reef scoring rods.

    



    public static class k_elevator {

        public static final Angle k_mag_sensor_offset = Rotations.of(-0.104004);
        public static final double k_sensor_to_mechanism =1/1.486486; // sensor rotations to mechanism travel ratio
        public static final double k_rotor_to_sensor = 11.71*5.5;


        public static final Angle k_min_rot = Rotations.of(0);
        public static final Angle k_max_rot = Rotations.of(0.98);


        // map points to convert from rotations to lengths of the elevator,
        // these should stay constant if the elevator is not modified
        // can be used as verification points to ensure the elevator is working correctly
        // if verification fails, then the elevator should come back into spec if re-mesured,
        // whithout need to change the scoring goal heights

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

        public static final Angle k_stowed =  Rotation.of(0.08);//0.08
        public static final Angle k_load =  Rotation.of(0.1);//0.08

        public static Angle k_coral_level_sense_postion_1 = Rotations.of(0.04); //trought
        public static Angle k_coral_level_sense_postion_2 = Rotations.of(0.14); // level 2
        public static Angle k_coral_level_sense_postion_3 = Rotations.of(0.36); // level 3
        public static Angle k_coral_level_sense_postion_4 = Rotations.of(0.95); // level 4


        // elevator goal leghts




        // physical Characteristics of robot/ verifcation only
        public static final Distance min_axel_height = Inches.of(12);
        public static final Distance max_axel_height = Inches.of(78);

        public static final Mass k_carrage_mass = Kilogram.of(13);



        // elevator motor constants

        public static final double k_current_limit = 70;
        // Slot 0 elevator pid gains
        public static final double k_0_ks = 0; // output to overcome static friction
        public static final double k_0_kp = 160; // proportional
        public static final double k_0_ki = 10; // integral
        public static final double k_0_kd = 120; //derivative
        public static final double k_0_kg = 6; //gravity; minimum ampage(?) for movement to account for opposing forces
        public static double k_0_kff = 3;
        public static double k_0_kff_offset = 0;
        //slot 0 motion magic expo gains
        public static final double k_0_kV = 3; // voltage required to maintain a given velocity, in V/rps
        public static final double k_0_kA = 0.5; // voltage required to apply a given acceleration, in V/(rps/s)
        public static final double k_0_cruiseVel = 0.4; // used for both mm and mm_expo
        
        // slot 0 "normal" motion magic gains
        public static final double k_0_Acceleration =10; //noma
        public static final double k_0_jerk = 10;

        


        
    }

    public static class k_claw{

    }

    public static class k_intake{

    }

    public static class k_climber{

    }

    public static class k_vision{

        public static final String k_front_camera_name = "front_camera";

        
        public static final Transform3d k_front_camera_offset = new Transform3d(
            new Translation3d(Inches.of(0.25), Inches.of(13), Inches.of(32.87)), 
            new Rotation3d(new Quaternion(0.349066 , 1, 0, 0))
            );

        public static final String k_rear_camera_name = "rear_camera";
        public static final Transform3d k_rear_camera_offset =  new Transform3d(
            new Translation3d(Inches.of(0.25), Inches.of(13), Inches.of(32.87)), 
            new Rotation3d(new Quaternion(0.349066 , 1, 0, 0))
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
