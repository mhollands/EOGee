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
nut_width = 5.25;
nut_height = 6.25;
nut_depth = 2.25;
difference()
{
    union()
    {
        difference()
        {
            cube([pcb_width, pcb_height, housing_thickness+pcb_standoff]);
            translate([pcb_standoff_width,pcb_standoff_width,housing_thickness])
            {   
                cube([pcb_width-2*pcb_standoff_width, pcb_height-2*pcb_standoff_width, pcb_standoff]);
            }
        }
        translate([mounting_hole_inset,mounting_hole_inset,0])cylinder(r=mounting_hole_diameter/2+pcb_standoff_width, h = pcb_standoff + housing_thickness);
        translate([pcb_width-mounting_hole_inset,mounting_hole_inset,0])cylinder(r=mounting_hole_diameter/2+pcb_standoff_width, h = pcb_standoff + housing_thickness);
        translate([pcb_width-mounting_hole_inset,pcb_height-mounting_hole_inset,0])cylinder(r=mounting_hole_diameter/2+pcb_standoff_width, h = pcb_standoff + housing_thickness);
        translate([mounting_hole_inset,pcb_height-mounting_hole_inset,0])cylinder(r=mounting_hole_diameter/2+pcb_standoff_width, h = pcb_standoff + housing_thickness);
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
    translate([pcb_width-mounting_hole_inset,mounting_hole_inset,0])
    {
        cylinder(r=mounting_hole_diameter/2+hole_tolerance, h = pcb_standoff + housing_thickness);
        cube([nut_width, nut_height, nut_depth*2], center=true);
    }
    translate([mounting_hole_inset,mounting_hole_inset,0])
    {
        cylinder(r=mounting_hole_diameter/2+hole_tolerance, h = pcb_standoff + housing_thickness);
        cube([nut_width, nut_height, nut_depth*2], center=true);
    }    
}