// Component behaviours (think in terms of state)


movement(waypoint).
+!waypoint(Location)
    :   atLocation(Location,_)
    <-  drive(stop).

+!waypoint(Location)
    :   nearLocation(Location,_)	
    <-  drivexy(0.1,0).

+!waypoint(Location)
    <-  drivexy(0.5,0).
	

