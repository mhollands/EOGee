$fn = 50;
pcb_width = 61.976;
pcb_height = 62.611;
housing_height = 20;
housing_thickness = 2;
mounting_hole_inset = 3.556;
mounting_hole_keepout = 4;
mounting_hole_diameter = 3.2;
bnc_width = 14.5;
bnc_height = 17.5;
snap_hole_width = 16;
snap_hole_height = 9;
snap_hole_separation = 20.32;
module volume()
{
    difference()
    {
        cube([pcb_width, pcb_height, housing_height]);
        //translate([0,0,0]) cylinder(r=mounting_hole_inset+mounting_hole_keepout, h=housing_height);
        //translate([pcb_width,0,0]) cylinder(r=mounting_hole_inset+mounting_hole_keepout, h=housing_height);
        translate([pcb_width,pcb_height,0]) cylinder(r=mounting_hole_inset+mounting_hole_keepout, h=housing_height);
        translate([0,pcb_height,0]) cylinder(r=mounting_hole_inset+mounting_hole_keepout, h=housing_height);
    }
}

x_scale_ratio = 1 - 2*housing_thickness/pcb_width;
y_scale_ratio = 1 - 2*housing_thickness/pcb_height;
difference()
{
    union()
    {
        translate([-pcb_width/2, -pcb_height/2, 0])
        {
            volume();
            difference()
            {
                cube([pcb_width, pcb_height, housing_thickness]);
                translate([mounting_hole_inset,mounting_hole_inset,0]) cylinder(r=mounting_hole_diameter/2, h=housing_height);
                translate([pcb_width-mounting_hole_inset,mounting_hole_inset,0]) cylinder(r=mounting_hole_diameter/2, h=housing_height);
                translate([pcb_width-mounting_hole_inset,pcb_height-mounting_hole_inset,0]) cylinder(r=mounting_hole_diameter/2, h=housing_height);
                translate([mounting_hole_inset,pcb_height-mounting_hole_inset,0]) cylinder(r=mounting_hole_diameter/2, h=housing_height);
            }
        }
    }
    scale([x_scale_ratio,y_scale_ratio,1])
    {
        translate([-pcb_width/2, -pcb_height/2]) volume() volume();
    }
    translate([0,pcb_height/2, bnc_height/2]) cube([bnc_width, pcb_height/2, bnc_height], center=true);
    translate([0,-pcb_height/2, snap_hole_height/2]) cube([snap_hole_width, pcb_height/2, snap_hole_height], center=true);
    translate([snap_hole_separation,-pcb_height/2, snap_hole_height/2]) cube([snap_hole_width, pcb_height/2, snap_hole_height], center=true);
        translate([-snap_hole_separation,-pcb_height/2, snap_hole_height/2]) cube([snap_hole_width, pcb_height/2, snap_hole_height], center=true);
    translate([0,-pcb_height/2, snap_hole_height/2]) cube([snap_hole_width, pcb_height/2, snap_hole_height], center=true);
}