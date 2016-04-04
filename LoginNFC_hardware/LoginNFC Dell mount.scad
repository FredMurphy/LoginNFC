$fn=50;

// Test block
translate([-5,-2,0])
    cube ([20,30,3.5]);

// Tab to fit in Dell monitor slot
translate([-1.5,0,-3])
    cube ([3,21.5,3.5]);

translate([-2.5,1,-4.75])
    minkowski()
    {
        cube ([7-2,21.5-2,3-2]);
        cylinder(r=1,h=1);
    }
