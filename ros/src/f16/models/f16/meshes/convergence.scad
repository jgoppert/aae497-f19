wingspan = 1.0;
length = 1.2;

module wing() {
    linear_extrude(height=0.05) {
        polygon([
            [0, 0],
            [length, wingspan],
            [length, 0]]);
    }
}

rotate([0, 90, 0]) rotate_extrude() polygon([
    [0, 0],
    [0.2*wingspan, length/4],
    [0, length]]);

wing();
mirror([0, 1, 0]) wing();