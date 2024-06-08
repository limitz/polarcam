
rotate([30,30,30])
{
    color("silver")
        cube([180,180,1], center=true);

    color([1.0,0,0,0.5])    
        rotate([0,90, 30])
            intersection() 
            {
                translate([-100,0,0]) 
                    cube([200,200,200], center=true);
                cylinder(.1, r=100, center=true);
            }
            
    color([0.0,1,0,0.5])    
        rotate([0,90, -40])
            intersection() 
            {
                translate([-100,0,0]) 
                    cube([200,200,200], center=true);
                cylinder(.1, r=100, center=true);
            }
}
