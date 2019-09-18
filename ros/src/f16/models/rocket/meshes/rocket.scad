$fn = 20; // use 20 sides for circle
l = 3; // length, m
r = 0.1; // radius, m
n = 0.2; // nose length
fh = 0.02; // fin thickness
fl = 0.5; // fin length

module nose_cone() {
    color([0.3, 0.3, 0.3]) translate([0, 0, l-n]) cylinder(h=n, r1=r, r2=0);
}

module body() {
    color([0.7, 0.7, 0.7]) cylinder(h=l-n, r=r);
}

module fin() {
    color([0.3, 0.3, 0.3]) rotate([90, 0, 0]) linear_extrude(fh, center=true) polygon([
    [0, fl],
    [fl, 0],
    [0, 0]
    ]);
}

module plume() {
    color([1, 1, 0, 0.5]) scale([r*0.5, r*0.5, 1]) 
    translate([0, 0, 0.5]) sphere(r=1);
}

part="assembly";

if (part == "assembly") {
    nose_cone();
    body();
    fin();
    rotate([0, 0, 90]) fin();
    rotate([0, 0, -90]) fin();
    rotate([0, 0, 180]) fin();
    plume();
} else if (part == "nose_cone") {
    nose_cone();
} else if (part == "body") {
    body();
} else if (part == "fin") {
    fin();
} else if (part =="plume") {
    plume();
}