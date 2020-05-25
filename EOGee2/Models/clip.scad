$fn = 50;
pcb_width = 78.994;
pcb_height = 60.706;
housing_thickness = 2.5;
mounting_hole_inset = 3.302;
mounting_hole_keepout = 4;
mounting_hole_diameter = 3.25;
hole_tolerance = 0.2;
pcb_standoff = 2;
pcb_standoff_width = 1.5;
nut_width = 5.75;
nut_height = 6.75;
nut_depth = 3;

clip_gap = 5;
clip_hinge_thickness = 10;
clip_screw_separation = 40;

thickness_tolerance = 0.4;

difference()
{
    union()
    {
        cube([pcb_width, pcb_height/2, housing_thickness]);
        translate([0,0,housing_thickness]) cube([pcb_width, clip_hinge_thickness, clip_gap - 2*thickness_tolerance]);
    }
    translate([pcb_width/2-clip_screw_separation/2,clip_hinge_thickness/2,0])
    {
        cylinder(r=mounting_hole_diameter/2+hole_tolerance, h = clip_gap + housing_thickness);
        cube([nut_width, nut_height, nut_depth*2], center=true);
    }
    translate([pcb_width/2+clip_screw_separation/2,clip_hinge_thickness/2,0])
    {
        cylinder(r=mounting_hole_diameter/2+hole_tolerance, h = clip_gap + housing_thickness);
        cube([nut_width, nut_height, nut_depth*2], center=true);
    }
}