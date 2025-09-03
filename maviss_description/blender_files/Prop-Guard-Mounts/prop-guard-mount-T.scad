// Four tube sockets (no base) with TRUE M2 clearance holes in each
// Directions: Right (+X), Back (+Y), Front (-Y), Down (-Z)

// --- Tube/socket params ---
tube_id        = 5;       // carbon fiber tube OD
clearance      = 0.15;    // PLA slip-fit
socket_wall    = 2;
socket_len     = 20;

// --- Screw hole params ---
m2_clearance_d   = 2.2;                  // M2 free clearance
screw_depth      = socket_wall + 1.2;    // bite in from surface
hole_axial_pos   = 6;                    // mm along the socket from the root (x=0)

// --- Quality ---
$fn = 64;

// Derived
socket_id = tube_id + clearance;
socket_od = socket_id + 2*socket_wall;
r_out     = socket_od/2;
r_in      = socket_id/2;

// ============= Socket bodies (no holes) =============

// Hollow cylinder along +X, starting at x=0 → x=len
module socket_xy_solid(angle_deg, len) {
  rotate([0,0,angle_deg]) {
    // build a Z cylinder (0..len) and rotate into +X
    rotate([0,90,0])
      difference() {
        cylinder(h=len,   r=r_out, center=false);
        translate([0,0,-0.1]) cylinder(h=len+0.2, r=r_in, center=false);
      }
  }
}

// Vertical socket pointing DOWN (occupies z ∈ [-len, 0])
module socket_down_solid(len) {
  translate([0,0,-len])
    difference() {
      cylinder(h=len, r=r_out, center=false);
      translate([0,0,-0.1]) cylinder(h=len+0.2, r=r_in, center=false);
    }
}

// ============= Hole cutters (just the holes) =============

// Vertical hole from the "top" of a horizontal socket.
// We rotate the cutter with the same Z‑rotation as the socket so it stays aligned.
module hole_xy(angle_deg) {
  rotate([0,0,angle_deg])
    translate([hole_axial_pos+6, 0, r_out - screw_depth])
      cylinder(d=m2_clearance_d, h=screw_depth + 0.4, center=false);
}

// Side hole into the down‑facing socket from +X
module hole_down(len) {
  translate([r_out - screw_depth, 0, -len/2-1])  // mid-height of the down socket
    rotate([0,90,0]) // drill along +X
      cylinder(d=m2_clearance_d, h=screw_depth + 0.4, center=false);
}

// ============= Build: union sockets, then subtract holes =============
difference() {
  union() {
    // Right (+X)
    //socket_xy_solid(0,   socket_len);
    // Back (+Y)
    socket_xy_solid(90,  socket_len);
    // Front (-Y)
    socket_xy_solid(-90, socket_len);
    // Down (-Z)
    socket_down_solid(socket_len);
  }

  // Cut M2 holes in ALL sockets
  hole_xy(0);      // Right
  hole_xy(90);     // Back
  hole_xy(-90);    // Front
  hole_down(socket_len); // Down
}
