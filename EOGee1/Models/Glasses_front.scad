$fn = 100;

glasses_width=143;
glasses_height = 44;
nose_width_top = 4;
nose_width_bottom = 17.5;
nose_height = 30;
rim_thickness = 6;
rim_depth = 3;

arm_join_depth = 3;
arm_join_height = 20;
arm_join_thickness = 10;

frame_joint_width = 5;
frame_joint_height= 10;

module rim_outline()
{
    outer_points = [  [0,0], 
                [glasses_width/2, 0],
                [glasses_width/2,-glasses_height],
                [nose_width_bottom, -glasses_height],
                [nose_width_top, -glasses_height + nose_height],
                [0, -glasses_height + nose_height]];

    inner_points = [    [nose_width_top + rim_thickness, -rim_thickness],
                        [glasses_width/2 - rim_thickness, -rim_thickness],
                        [glasses_width/2 - rim_thickness, -glasses_height+rim_thickness],
                        [nose_width_bottom + rim_thickness, -glasses_height + rim_thickness],
                        [nose_width_top + rim_thickness, -glasses_height + nose_height]];

    difference()
    {
        polygon(points = outer_points);
        polygon(points = inner_points);
    }
}

module half_frame()
{
    // Draw rim
    linear_extrude(rim_depth)
    {
        rim_outline();
    }

    // Make the arm mount
    // Move to edge
    translate([glasses_width/2 - rim_thickness, -arm_join_thickness, rim_depth])
    {
        //Make two holes in mount
        difference()
        {
            cube([arm_join_depth, arm_join_thickness, arm_join_height]);
            translate([0,arm_join_thickness/2, arm_join_height/3])
            {
                rotate([0,90,0])
                {
                    cylinder(r=2, h=arm_join_depth);
                }
            }
            translate([0,arm_join_thickness/2, arm_join_height*2/3])
            {
                rotate([0,90,0])
                {
                    cylinder(r=2, h=arm_join_depth);
                }
            }
        }
    }
}

difference()
{
    half_frame();
    translate([0,-frame_joint_height,rim_depth/2])
    {
        cube([frame_joint_width,frame_joint_height,rim_depth/2]);
    }
}