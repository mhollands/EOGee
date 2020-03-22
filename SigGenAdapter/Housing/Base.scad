$fn = 50;
pcb_width = 61.976;
pcb_height = 62.611;
housing_height = 20;
housing_thickness = 3;
mounting_hole_inset = 3.556;
mounting_hole_keepout = 4;
mounting_hole_diameter = 3.2;
hole_tolerance = 0.2;
bnc_width = 14.5;
bnc_height = 17.5;
snap_hole_width = 16;
snap_hole_height = 9;
snap_hole_separation = 20.32;
pcb_standoff = 3;
pcb_standoff_width = 1;
nut_width = 5;
nut_height = 6;
nut_depth = 2.25;
pcb_thickness = 1.6;
difference()
{
    union()
    {
        difference()
        {
            cube([pcb_width, pcb_height, housing_thickness+pcb_standoff]);
            translate([housing_thickness,housing_thickness,housing_thickness])
            {   
                cube([pcb_width-2*housing_thickness, pcb_height-2*housing_thickness, pcb_standoff]);
            }
        }
        translate([mounting_hole_inset,mounting_hole_inset,0])cylinder(r=mounting_hole_diameter/2+pcb_standoff_width, h = pcb_standoff + housing_thickness);
        translate([pcb_width-mounting_hole_inset,mounting_hole_inset,0])cylinder(r=mounting_hole_diameter/2+pcb_standoff_width, h = pcb_standoff + housing_thickness);
        translate([pcb_width-mounting_hole_inset,pcb_height-mounting_hole_inset,0])cylinder(r=mounting_hole_diameter/2+pcb_standoff_width, h = pcb_standoff + housing_thickness);
        translate([mounting_hole_inset,pcb_height-mounting_hole_inset,0])cylinder(r=mounting_hole_diameter/2+pcb_standoff_width, h = pcb_standoff + housing_thickness);
        
        translate([mounting_hole_inset,mounting_hole_inset,0]) cylinder(r=mounting_hole_diameter/2-hole_tolerance, h = pcb_standoff + housing_thickness + pcb_thickness);
        translate([pcb_width-mounting_hole_inset,mounting_hole_inset,0]) cylinder(r=mounting_hole_diameter/2-hole_tolerance, h = pcb_standoff + housing_thickness + pcb_thickness);
    }
    translate([pcb_width-mounting_hole_inset,pcb_height-mounting_hole_inset,0])
    {
        cylinder(r=mounting_hole_diameter/2+hole_tolerance, h = pcb_standoff + housing_thickness);
        cube([nut_width, nut_height, nut_depth*2], center=true);
    }
    translate([mounting_hole_inset,pcb_height-mounting_hole_inset,0])
    {
        cylinder(r=mounting_hole_diameter/2+hole_tolerance, h = pcb_standoff + housing_thickness);
        cube([nut_width, nut_height, nut_depth*2], center=true);
    }
    
}