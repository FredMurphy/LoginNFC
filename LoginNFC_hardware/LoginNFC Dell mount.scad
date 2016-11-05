$fn=50;

l = 88.17;
w = 43.18;
t = 3;

x1 = 3.81;
x2 = 29.21;
x3 = 69.85;
y1 = 3.81;
y2 = 36.83;

module base() {

translate([-13.24+1,-1.27+1,0])
    minkowski()
    {
        cube ([l-2,w-2,t-1]);
        cylinder(r=1,h=1);
    }

}


    translate([6.5,37,0]) {
    // shaft
        translate([0,-1.5,-4])
            cube ([20,3,4.5]);

    // tab
    translate([1,-2.51,-4.45])
        minkowski()
        {
            cube ([20-2,7-2,2.5-2]);
            sphere(r=1);
        }
    }
    
// Tab to fit in Dell monitor slot
    difference () {
        base();
        translate([x1,y1,0])
            cylinder(r=1.2,h=t);
        translate([x1,y2,0])
            cylinder(r=1.2,h=t);
        translate([x2,y1,0])
            cylinder(r=1.2,h=t);
        translate([x2,y2,0])
            cylinder(r=1.2,h=t);
        translate([x3,y1,0])
            cylinder(r=1.2,h=t);
        translate([x3,y2,0])
            cylinder(r=1.2,h=t);
        
        translate([-11,25.535,-0.1])
            cube([11,8.6,t+0.2]);        
}


