$fn = 50;
base_diameter = 7.2;
base_thickness = 3.5;
screw_depth = 3;
screw_diameter = 3;
slot_thickness = 1.6;
slot_top_offset = 0.5;
tip_length = 3;
tip_base_diameter = 3.65;
tip_top_diameter = 3.9;
difference()
{
    cylinder(r=base_diameter/2, h = base_thickness);
    cylinder(r=screw_diameter/2, h = screw_depth);
    translate([-base_diameter/2, -slot_thickness/2, base_thickness-slot_thickness-slot_top_offset]) cube([base_diameter, slot_thickness, slot_thickness]);
}

translate([0,0,base_thickness]) cylinder(r1=tip_base_diameter/2,r2=tip_top_diameter/2, h = tip_length);