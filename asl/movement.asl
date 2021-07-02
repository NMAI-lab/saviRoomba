
movement(waypoint).
+!waypoint(Location)
    :   at(Location)
    <-  drive(stop).

	
+!waypoint(Location)
    :   beacon(Location, Range, Direction)
        & (not at(Location))
    <-  drive(Direction).

at(Location)
    :-  position(X,Y) 
		& locationName(Location,[X,Y]).

