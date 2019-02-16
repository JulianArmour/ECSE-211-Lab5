package ca.mcgill.ecse211.lab5.navigator;

import ca.mcgill.ecse211.lab5.odometer.Odometer;

public class SearchNavigator {

    private Odometer odometer;
    private MovementController movementController;
    private double llX;
    private double llY;
    private double urX;
    private double urY;

    public SearchNavigator(Odometer odometer, MovementController movementController, 
                           double llX, double llY, double urX, double urY) 
    {
        this.odometer = odometer;
        this.movementController = movementController;
        this.llX = llX;
        this.llY = llY;
        this.urX = urX;
        this.urY = urY;
    }
    
    
    
}
