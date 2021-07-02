
movement(waypoint).
+!waypoint(Location)
    :   at(Location)
    <-  drive(stop).

	
// irWall(A,B)
// beacon(Mac,Range)
// position(X,Y)
// at(Location)
// range(A,B,Range)
	
+!waypoint(Location)
    :   beacon(Location, Range, Direction)
        & (not at(Location))
    <-  drive(Direction).

at(Location)
    :-  position(X,Y) 
		& locationName(Location,[X,Y]).

