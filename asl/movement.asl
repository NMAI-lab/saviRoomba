// Component behaviours (think in terms of state)


movement(waypoint).
+!waypoint(Location)
    :   atLocation(Location,Range)
    <-  .broadcast(tell, waypoint(atLocation(Location,Range)));
		drive(stop).

+!waypoint(Location)
    :   nearLocation(Location,Range)	
    <-  .broadcast(tell, waypoint(nearLocation(Location,Range)));
		drivexy(0.1,0);
		!waypoint(Location).

+!waypoint(Location)
    <-	.broadcast(tell, waypoint(driving(Location)));
		drivexy(0.5,0);
		!waypoint(Location).
	

