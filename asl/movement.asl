
movement(waypoint).
+!waypoint(Location)
    :   at(Location)
    <-  drive(stop).

	
// irWall(Distance,Angle)
// beacon(Mac,Range)
// position(X,Y)
// at(Location)
// range(A,B,Range)
	
+!waypoint(Location)
    :   (not at(Location))
    <-  drive(forward).

at(Location)
    :-  position(X,Y) 
		& locationName(Location,[X,Y]).

