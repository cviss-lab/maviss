// Plate with two tube sockets aimed toward rectangle corners, slightly recessed into the plate (for 5 mm CF tubes) and an M2 hole in the base
// All units in mm

// --- Plate ---
length = 20; // X (mm)
width  = 10; // Y (mm)
thick  = 5;  // Z (mm)

// --- Tube/socket params ---
tube_id        = 5;      // carbon fiber tube outer diameter
clearance      = 0.15;  // fit clearance for insertion
socket_wall    = 2;      // wall thickness around the tube
socket_len     = 20;     // length of each socket along its axis
recess_depth   = 2;      // how far sockets sit inside the plate

// --- Hole params ---
m2_diameter      = 3.2;   // base-plate hole diameter (~2.2 for true M2 clearance)
m2_top_diameter  = 2.2;   // true M2 clearance for the vertical hole at socket top
screw_depth      = socket_wall + 1.2; // how deep the top hole goes down from socket top (tweak as needed)

// --- Quality settings ---
$fn = 64; // ensure circles are smooth

socket_id = tube_id + clearance;      // inner diameter of socket
socket_od = socket_id + 2*socket_wall; // outer diameter
r_out     = socket_od/2;
r_in      = socket_id/2;

module plate() {
  difference() {
    cube([length, width, thick], center=false);
    // M2 hole at center of plate (offset 5 mm from geometric center in X per your original)
    translate([length/2-6, width/2, 0])
      cylinder(d=m2_diameter, h=thick+0.1, center=false);
  }
}

// Generic horizontal socket pointing at an in‑plane angle (deg) relative to +X
// Centered at (xpos, ypos); partially recessed into the plate
module socket_dir(angle_deg, len, xpos, ypos) {
  translate([xpos, ypos, thick + r_out - recess_depth])
    rotate([0,0,angle_deg])       // rotate around Z to desired heading in XY
      rotate([0,90,0])            // lay cylinder horizontal (axis along +X before Z-rot)
        difference() {
          cylinder(h=len, r=r_out, center=true);
          cylinder(h=len+0.2, r=r_in, center=true);
        }
}

// Vertical M2 hole drilled from the very top of the socket downward by screw_depth
// Place at the same (xpos, ypos) used for the socket.
module socket_top_hole(xpos, ypos) {
  // Top of socket in world Z is: (socket center Z) + r_out
  // socket center Z = thick + r_out - recess_depth
  z_top = thick + 2*r_out - recess_depth;
  translate([xpos, ypos, z_top - screw_depth])
    cylinder(d=m2_top_diameter, h=screw_depth + 0.2, center=false);
}

// --- Tube headings (exactly 90° apart) ---
//ang_A = atan2(width,  length-5); // base direction
//ang_B = ang_A + 90;              // exactly perpendicular to ang_A
ang_A = 45;
ang_B = -45;
// --- Placement ---
plate_centre = [length/2+7, width/2];

// Socket XY positions (match the ones used below so we can reuse for the holes)
sock1_pos = [plate_centre[0]+3, plate_centre[1]+5];
sock2_pos = [plate_centre[0]+3, plate_centre[1]-5];

difference() {
  // 1) Build plate + sockets
  union() {
    plate();
    //socket_dir(ang_A, socket_len, sock1_pos[0], sock1_pos[1]);
    socket_dir(ang_B, socket_len, sock2_pos[0], sock2_pos[1]);
  }

  // 2) Subtract vertical M2 holes "on top" of each socket
  socket_top_hole(sock1_pos[0]+1, sock1_pos[1]+1);
  socket_top_hole(sock2_pos[0]+1, sock2_pos[1]-1);
}

// Notes:
// - Increase `screw_depth` if you want the screw to reach the tube ID further.
//   It only needs to pass the outer wall (≈ socket_wall) to bite the tube.
// - If you want a threaded boss, keep this hole at 1.6–1.7 mm and tap it;
//   otherwise 2.2 mm is good for free clearance.
// - To move the holes with the sockets, only adjust sock1_pos / sock2_pos once.
