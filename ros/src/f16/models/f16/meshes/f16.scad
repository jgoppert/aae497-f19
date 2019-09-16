$fn=10;
length = 15.06;
wingspan = 9.96;

chord = length*0.35;
taper = 0.25;

tail_span = wingspan*0.5;
tail_chord = chord*0.7;
tail_taper = 0.35;

vert_span = tail_span;
vert_chord = tail_chord;
vert_taper = 0.2;

module wing(chord, wingspan, taper) {
    color([0.6, 0.6, 0.6]) translate([-chord, 0]) rotate([90, 0, 180]) linear_extrude(height=wingspan/2, scale=taper) polygon([
        [0, 0],
        [-chord*0.75, 0.05*chord],
        [-chord, 0],
        [-chord*0.75, -0.025*chord]
    ]);
}

module wing_sym(chord, wingspan, taper) {
    color([0.6, 0.6, 0.6]) translate([-chord, 0]) rotate([90, 0, 180]) linear_extrude(height=wingspan/2, scale=taper) polygon([
        [0, 0],
        [-chord*0.75, 0.05*chord],
        [-chord, 0],
        [-chord*0.75, -0.05*chord]
    ]);
}

module airframe_half() {
    translate([-length*0.35, 0]) wing(chord, wingspan, taper);
    translate([-length*0.7, 0]) wing(tail_chord, tail_span, tail_taper);
    translate([-length*0.7, 0]) rotate([90, 0, 0]) 
        wing(vert_chord, vert_span, vert_taper);
}

module body() {
    color([0.6, 0.6, 0.6]) rotate([0, -90, 0]) rotate_extrude() polygon([
        [0,0],
        [0.07*wingspan,0.2*length],
        [0.09*wingspan,0.4*length],
        [0.09*wingspan,0.6*length],
        [0.07*wingspan,0.95*length],
        [0.05*wingspan,length],
        [0,0.95*length]
    ]);
}

module canopy() {
    color([0.6, 0.6, 0.8, 0.5]) translate([-0.3*length, 0, 0.044*length]) rotate([0, 4, 0]) scale([0.13*length, 0.039*length, 0.026*length]) difference() {
       sphere(1);
       translate([0, 0, -1.5]) cube([3, 3, 3], center=true);
    }
}

module airframe_no_cuts() {
    body();
    translate([-length*0.35, 0]) wing(chord, wingspan, taper);
    mirror([0, 1, 0]) translate([-length*0.35, 0]) wing(chord, wingspan, taper);
    translate([-length*0.7, 0]) wing(tail_chord, tail_span, tail_taper);
    mirror([0, 1, 0]) translate([-length*0.7, 0]) wing(tail_chord, tail_span, tail_taper);
    translate([-length*0.7, 0]) rotate([90, 0, 0]) 
        wing_sym(vert_chord, vert_span, vert_taper);
}

module aileron_cut(s) {
    translate([-0.7*length, wingspan/4, 0]) scale([s, s, 1]) cube([length*0.1, wingspan*0.2, length*0.1], center=true);
}

module elevator_cut(s) {
    translate([-0.9*length, tail_span*0.37, 0]) scale([s, s, 1]) cube([length*0.3, wingspan*0.2, length*0.1], center=true);
}

module rudder_cut(s) {
    translate([-length*0.95, 0, length*0.1]) scale([s, s, 1]) cube([length*0.1, wingspan*0.2, length*0.1], center=true);
}

module airframe() {
    difference() {
        airframe_no_cuts();
        aileron_cut(1);
        mirror([0, 1, 0]) aileron_cut(1);
        elevator_cut(1);
        mirror([0, 1, 0]) elevator_cut(1);
        rudder_cut();
        translate([-4.5, 0, 0.5]) cube([2, 0.8, 2], center=true); // cockpit
    }
}

module aileron() {
    color("red") render() intersection() {
        aileron_cut(0.99);
        airframe_no_cuts();
    }
}

module elevator() {
    color("red") render() intersection() {
        elevator_cut(0.99);
        airframe_no_cuts();
    }
}

module rudder() {
    color("red") render() intersection() {
        rudder_cut(0.99);
        airframe_no_cuts();
    }
}

module assembly() {
    airframe();
    aileron();
    mirror([0, 1, 0]) aileron();
    rudder();
    elevator();
    mirror([0, 1, 0]) elevator();
    canopy();
}

part = "assembly";

if (part == "airframe") {
    airframe();
} else if (part == "canopy") {
    canopy();
} else if (part == "left_aileron") {
    aileron();
} else if (part == "left_elevator") {
    elevator();
} else if (part == "right_aileron") {
    mirror([0, 1, 0]) aileron();
} else if (part == "right_elevator") {
    mirror([0, 1, 0]) elevator();
} else if (part == "rudder") {
    rudder();
} else {
    assembly();
}
