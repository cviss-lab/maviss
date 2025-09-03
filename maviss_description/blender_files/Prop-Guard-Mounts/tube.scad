// 100 mm long, 5 mm diameter test piece for your sockets
// Set wall_thick = 0 for a solid rod, or >0 for a hollow tube.

// --- Params ---
len_mm     = 200;   // length (10 cm)
outer_d    = 5.0;   // outer diameter to match your CF tube
wall_thick = 0;     // 0 => solid; e.g. 1.0 => 1 mm wall (ID = outer_d - 2*wall_thick)
chamfer_h  = 0.4;   // small lead-in at each end

$fn = 128;

// --- Derived ---
inner_d = (wall_thick > 0) ? max(0, outer_d - 2*wall_thick) : 0;

// --- Main ---
module tube_or_rod(od, id, L, chamfer=0) {
  difference() {
    // body
    union() {
      // straight section
      translate([0,0,chamfer]) cylinder(d=od, h=L - 2*chamfer, center=false);
      // chamfers
      if (chamfer > 0) {
        // bottom chamfer
        cylinder(h=chamfer, d1=od-0.6, d2=od, center=false);
        // top chamfer
        translate([0,0,L - chamfer]) cylinder(h=chamfer, d1=od, d2=od-0.6, center=false);
      }
    }
    // hollow if requested
    if (id > 0) {
      translate([0,0,-0.1]) cylinder(d=id, h=L+0.2, center=false);
    }
  }
}

tube_or_rod(outer_d, inner_d, len_mm, chamfer_h);
