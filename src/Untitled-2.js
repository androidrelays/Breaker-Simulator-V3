scalev = 25.4; // scale of project (inches → mm)
scale([scalev, scalev, scalev]) {
    $fn = 50;

    // Dimensions
    lid_l = 10;   // length (X)
    lid_w = 6;    // top width (Y)
    lid_t = 3;    // height (Z)
    wallt = 1/8;  // wall thickness

    // Fillet radii
    lid_outrad = 3/8;
    lid_inrad  = 9/32;

    // Bolt coordinates
    hole1x=0;     hole1y=2.585;
    hole2x=0;     hole2y=-2.585;
    hole3x=4.585; hole3y=2.585;
    hole4x=4.585; hole4y=-2.585;
    hole5x=-4.585;hole5y=2.585;
    hole6x=-4.585;hole6y=-2.585;

    recess = 1/16*2;

    difference() {
        union() {
            // ====================================
            // Open-top/open-bottom tapered shell
            // ====================================
            open_taper_shell(lid_l, lid_w, lid_t, wallt);

            // --- Bolt standoffs ---
            translate([hole1x, hole1y, 0]) rotate([0,0,90])  keypost(lid_t-1/4,3/8,1/4);
            translate([hole2x, hole2y, 0]) rotate([0,0,-90]) keypost(lid_t-1/4,3/8,1/4);
            translate([hole3x, hole3y, 0]) rotate([0,0,45])  keypost(lid_t-1/4,3/8,1/4);
            translate([hole4x, hole4y, 0]) rotate([0,0,-45]) keypost(lid_t-1/4,3/8,1/4);
            translate([hole5x, hole5y, 0]) rotate([0,0,135]) keypost(lid_t-1/4,3/8,1/4);
            translate([hole6x, hole6y, 0]) rotate([0,0,-135])keypost(lid_t-1/4,3/8,1/4);
       // --- extra short keyposts (you can move these) ---
translate([ 4.6, 4.5, -1.3]) rotate([0, 0, 45]) keypost_short(0.25); // right corner
translate([ 0.0, 4.43, -1.3]) rotate([0, 0, 90]) keypost_short(0.25); // center
translate([-4.55, 4.5, -1.3]) rotate([0, 0, 135]) keypost_short(0.25); // left corner

        }

        // ===========================
        // Cutouts
        // ===========================
        translate([lid_l/2, lid_w/4.5, 0]) rotate([0,90,0]) amphenol1();
        translate([lid_l/2,-lid_w/4,0]) rotate([0,90,0])
            cylinder(h=0.5, r=0.83/2, center=true);
    }
}

// =============================================
// =============== MODULES =====================
// =============================================
// --- Recess panel cutout module ---
module banana_panel_recess(width=4.0, height=1.6, depth=1/8) {
    // width, height, and depth (in inches) of recess area
    cube([width, height, depth], center=true);
}

module open_taper_shell(lid_l, lid_w, lid_t, wallt) {
    taper_angle = atan(4/3); // ≈ 53.13°
    front_taper_out = 2.0;       // how far slanted wall extends forward
    front_vertical_out = 2.25;   // how far vertical lower part extends
    break_z = -lid_t * 0.25;     // where taper transitions to vertical
    top_z = lid_t / 2;
    bottom_z = -lid_t / 2;

    // --- Back wall (vertical)
    difference() {
        translate([0, -lid_w/2 + wallt/2, 0])
            cube([lid_l, wallt, lid_t], center=true);
        translate([-2.5, -lid_w/2 - 0.05, 0])
            rotate([90, 0, 0]) banana1();
    }

    // --- Front wall (tapered top + vertical bottom) and sides
    difference() {
        union() {
            // Tapered 3/4 section
            hull() {
                translate([0, lid_w/2 - wallt/2, top_z])
                    cube([lid_l, wallt, 0.001], center=true);
                translate([0, lid_w/2 + front_taper_out, break_z])
                    cube([lid_l, wallt, 0.001], center=true);
            }

            // Lower vertical section
            hull() {
                translate([0, lid_w/2 + front_taper_out, break_z])
                    cube([lid_l, wallt, 0.001], center=true);
                translate([0, lid_w/2 + front_vertical_out, bottom_z])
                    cube([lid_l, wallt, 0.001], center=true);
            }
            // --- Instantiate both sides ---
            side_wall(1);   // right wall
            side_wall(-1);  // left wall
        }

        // ============================================

        // ============================================
        // Banana jack recesses + holes (on slanted wall)
        // ============================================
        translate([0, lid_w/2 + 1.98, 0.4])      // position slightly forward and up
            rotate([-taper_angle+6, 0, 0]) {       // align to wall slope

                // 🔹 main recess (large 3×4 group)
                translate([-2.2, -.6, -0.7])     // shift slightly left and into wall
                    banana_panel_recess(width=3.8, height=2.8, depth=.125);

                // 🔹 sense group recess (small 3×3 group)
                translate([2.4, -.6, -.7])       // offset right/down a bit
                    banana_panel_recess(width=3.0, height=2.8, depth=.125);

                // 🔹 actual banana holes (also shown solid for positioning)
                banana_panel();
            }

        // --- Remove top ---
        translate([0, 0, lid_t/2 - 0.01])
            cube([lid_l*2, lid_w*2, lid_t], center=true);
        // --- Remove bottom ---
        translate([0, 0, -lid_t/2 + 0.01])
            cube([lid_l*2, lid_w*2, lid_t], center=true);
    }
}
 module side_wall(xsign = 1) {
    wall_thick = wallt;

    // Key Y and Z positions
    back_y   = -lid_w/2 + wallt/2;
    taper_y  =  lid_w/2 + front_taper_out;
    front_y  =  lid_w/2 + front_vertical_out;
    top_z    =  lid_t/2;
    mid_z    =  break_z;
    bot_z    =  bottom_z;

    // Lower polyhedron: vertical front to back
    outer_lower = [
        [xsign * lid_l/2, back_y,   top_z],   // top back
        [xsign * lid_l/2, taper_y,  mid_z],   // break/taper
        [xsign * lid_l/2, front_y,  bot_z],   // bottom front
        [xsign * lid_l/2, back_y,   bot_z]    // bottom back
    ];
    inner_lower = [
        [xsign * (lid_l/2 - wall_thick), back_y,   top_z],
        [xsign * (lid_l/2 - wall_thick), taper_y,  mid_z],
        [xsign * (lid_l/2 - wall_thick), front_y,  bot_z],
        [xsign * (lid_l/2 - wall_thick), back_y,   bot_z]
    ];
    polyhedron(
        points = concat(outer_lower, inner_lower),
        faces = [
            [0,1,2,3],        // outer face
            [4,5,6,7],        // inner face
            [0,4,5,1],        // top/back strip
            [1,5,6,2],        // tapered front strip
            [2,6,7,3],        // bottom strip
            [3,7,4,0]         // back strip
        ]
    );

    // Upper polyhedron: slanted/tapered front
    outer_upper = [
        [xsign * lid_l/2, back_y,   top_z],   // top back
        [xsign * lid_l/2, taper_y,  mid_z],   // break/taper
        [xsign * lid_l/2, front_y,  top_z]    // top front
    ];
    inner_upper = [
        [xsign * (lid_l/2 - wall_thick), back_y,   top_z],
        [xsign * (lid_l/2 - wall_thick), taper_y,  mid_z],
        [xsign * (lid_l/2 - wall_thick), front_y,  top_z]
    ];
    polyhedron(
        points = concat(outer_upper, inner_upper),
        faces = [
            [0,1,2],        // outer face
            [3,5,4],        // inner face
            [0,3,4,1],      // top/back strip
            [1,4,5,2],      // tapered front strip
            [0,2,5,3]       // top strip
        ]
    );
}
}

// --- Bolt standoff post ---
module keypost(hh, cdd, offd){
    difference() {
        union() {
            cylinder(hh, d=cdd, $fn=25, center=true);
            translate([cdd/2,0,0])
                cube([cdd/2+offd,cdd*0.5,hh],center=true);
        }
        cylinder(hh*1.01, d=0.180, $fn=25, center=true);
    }
}
// --- Short adjustable keypost with tab (same design as main posts) ---
module keypost_short(height=0.25, diam=0.375, tab_offset=0.25, hole_diam=0.18) {
    difference() {
        union() {
            // round post
            cylinder(h=height, d=diam, $fn=40, center=true);

            // rectangular connection tab
            translate([diam/2, 0, 0])
                cube([diam/2 + tab_offset, diam*0.5, height], center=true);
        }

        // center through-hole
        cylinder(h=height*1.2, d=hole_diam, $fn=30, center=true);
    }
}

// --- Amphenol connector cutout ---
module amphenol1(){
    $fn=50; bigholed=1.26; thick=0.5; shd=0.12; holespace=1.25;
    cylinder(h=thick, r=bigholed/2, center=true);
    translate([holespace/2,holespace/2,0]) cylinder(h=thick,r=shd/2,center=true);
    translate([holespace/2,-holespace/2,0]) cylinder(h=thick,r=shd/2,center=true);
    translate([-holespace/2,holespace/2,0]) cylinder(h=thick,r=shd/2,center=true);
    translate([-holespace/2,-holespace/2,0]) cylinder(h=thick,r=shd/2,center=true);
}
// --- Banana jack hole module ---
module banana1(){
    $fn=50;
    dscale=1;
    oversize=1.02;
    dround=0.47*dscale*oversize;
    dflat=0.42*dscale*oversize;
    hheight=0.4*dscale;

    difference(){
        cylinder(h=hheight,d=dround,center=true);
        union(){
            translate([0,dflat/2+(dround/2-dflat/2)/2,0])
                cube([dround,dround/2-dflat/2,hheight*1.01],center=true);
            translate([0,-(dflat/2+(dround/2-dflat/2)/2),0])
                cube([dround,dround/2-dflat/2,hheight*1.01],center=true);
        }
    }
}
// --- Banana jack hole group for tapered wall ---
// --- Banana jack hole group for tapered wall ---
// Top 3x4 block + bottom 3x3 block
module banana_panel() {

    // ===== First cluster: 3 rows × 4 columns =====
    translate([-2.2, -.5, -.75]) {
        // top row (4 holes)
        translate([-1.35,  0.8, 0]) banana1();
        translate([-0.45,  0.8, 0]) banana1();
        translate([ 0.45,  0.8, 0]) banana1();
        translate([ 1.35,  0.8, 0]) banana1();

        // middle row (4 holes)
        translate([-1.35,  0.0, 0]) banana1();
        translate([-0.45,  0.0, 0]) banana1();
        translate([ 0.45,  0.0, 0]) banana1();
        translate([ 1.35,  0.0, 0]) banana1();

        // bottom row (4 holes)
        translate([-1.35, -0.8, 0]) banana1();
        translate([-0.45, -0.8, 0]) banana1();
        translate([ 0.45, -0.8, 0]) banana1();
        translate([ 1.35, -0.8, 0]) banana1();
    }

    // ===== Second cluster: 3×3 set (Sense group) =====
    translate([1.5,.25, -.75]) {   // move slightly downward and left
        // row 1
        translate([0,  0.0, 0]) banana1();
        translate([0.9, 0.0, 0]) banana1();
        translate([1.8, 0.0, 0]) banana1();

        // row 2
        translate([0, -0.75, 0]) banana1();
        translate([0.9, -0.75, 0]) banana1();
        translate([1.8, -0.75, 0]) banana1();

        // row 3
        translate([0, -1.5, 0]) banana1();
        translate([0.9, -1.5, 0]) banana1();
        translate([1.8, -1.5, 0]) banana1();
    }
}

;

