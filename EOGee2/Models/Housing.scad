$fn = 50;
pcb_width = 78.994;
pcb_height = 60.706;
housing_thickness = 1.5;

jack_y_position = pcb_height/2;
jack_length = 46.706;
jack_height = 6;

housing_height = 12;

mounting_hole_inset = 3.302;
mounting_hole_keepout = 6;
mounting_hole_diameter = 3.25;
mounting_hole_thickness = 6.15; //selected to make 12mm screw fit perfectly

mount_block_width = mounting_hole_inset*2;
mount_block_height = mounting_hole_inset*2;

usb_y_position = 37.973;
usb_length = 9;
usb_height = 3;

LED_x_position = 55.372;
LED_length = 5.207;
LED_height = 2;

tp101_x = 35.2425;
tp102_x = 49.53;
tp103_x = 53.9115;
tp104_x = 27.305;
tp_z_position = housing_height/2;
tp_diameter = 3;

difference()
{
    union()
    {
        // Hollow housing
        difference()
        {
            cube([pcb_width, pcb_height, housing_height]);
            translate([housing_thickness,housing_thickness,0]) cube([pcb_width-housing_thickness*2, pcb_height-housing_thickness*2, housing_height-housing_thickness]);
        }

        // Add mount blocks
        cube([mount_block_width, mount_block_height, housing_height]);
        translate([pcb_width-mount_block_width, 0, 0]) cube([mount_block_width, mount_block_height, housing_height]);
        translate([pcb_width-mount_block_width, pcb_height-mount_block_height, 0]) cube([mount_block_width, mount_block_height, housing_height]);
        translate([0 , pcb_height-mount_block_height, 0]) cube([mount_block_width, mount_block_height, housing_height]);
    }
    // Screw holes
    translate([mounting_hole_inset,mounting_hole_inset,0]) cylinder(r=mounting_hole_diameter/2, h=housing_height);
    translate([pcb_width-mounting_hole_inset,mounting_hole_inset,0]) cylinder(r=mounting_hole_diameter/2, h=housing_height);
    translate([pcb_width-mounting_hole_inset,pcb_height-mounting_hole_inset,0]) cylinder(r=mounting_hole_diameter/2, h=housing_height);
    translate([mounting_hole_inset,pcb_height - mounting_hole_inset,0]) cylinder(r=mounting_hole_diameter/2, h=housing_height);
    
    // Screw hole keepout
    translate([mounting_hole_inset,mounting_hole_inset,mounting_hole_thickness]) cylinder(r=mounting_hole_keepout/2, h=housing_height);
    translate([pcb_width-mounting_hole_inset,mounting_hole_inset,mounting_hole_thickness]) cylinder(r=mounting_hole_keepout/2, h=housing_height);
    translate([pcb_width-mounting_hole_inset,pcb_height-mounting_hole_inset,mounting_hole_thickness]) cylinder(r=mounting_hole_keepout/2, h=housing_height);
    translate([mounting_hole_inset,pcb_height - mounting_hole_inset,mounting_hole_thickness]) cylinder(r=mounting_hole_keepout/2, h=housing_height);
    
    // USB cut-out
    translate([pcb_width-housing_thickness, usb_y_position-usb_length/2, 0]) cube([housing_thickness, usb_length, usb_height]);
    // LED cut-out
    translate([LED_x_position-LED_length/2, pcb_height-housing_thickness, 0]) cube([LED_length, housing_thickness, LED_height]);
    // Jack cut-out
    translate([0, jack_y_position-jack_length/2, 0]) cube([housing_thickness, jack_length, jack_height]);
    
    // Test points
    translate([tp101_x, 0, tp_z_position]) rotate([-90,0,0]) cylinder(r=tp_diameter/2, h = housing_thickness);
    translate([tp102_x, 0, tp_z_position]) rotate([-90,0,0]) cylinder(r=tp_diameter/2, h = housing_thickness);
    translate([tp103_x, 0, tp_z_position]) rotate([-90,0,0]) cylinder(r=tp_diameter/2, h = housing_thickness);
    translate([tp104_x, 0, tp_z_position]) rotate([-90,0,0]) cylinder(r=tp_diameter/2, h = housing_thickness);
    
    // Text
    translate([pcb_width/2, pcb_height/2, housing_height-housing_thickness/2]) linear_extrude(height = housing_thickness/2) text("EOGee 2", size = 5, halign="center", valign="center");
    translate([pcb_width-2, pcb_height/2, housing_height-housing_thickness/2]) rotate([0,0,90]) linear_extrude(height = housing_thickness/2) text("projects.matthollands.com", size = 2, halign="center", valign="bottom");
}