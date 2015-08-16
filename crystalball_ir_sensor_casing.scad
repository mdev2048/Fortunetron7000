tgtDetail = 90;
ledRadius = 6;
module base() {
    cube([30,10,5], center=true);
    translate([-12.5,0,-8])
        cube([5,10,11],center=true);
    translate([12.5,0,-8])
        cube([5,10,11],center=true);
    translate([0,0,-8])
        cube([5,10,11],center=true);
}
difference() {
    base();
    translate([-6,0,0]) cylinder(d=ledRadius, h=6, $fn = tgtDetail, center=true);
    translate([6,0,0]) cylinder(d=ledRadius, h=6, $fn = tgtDetail, center=true);
    
}
