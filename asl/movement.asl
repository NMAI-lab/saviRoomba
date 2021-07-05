
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
		& move(Direction)		
    <-  drive(Direction);
		!waypoint(Location).

+!waypoint(Location)
    <-  !waypoint(Location).
		
at(Location)
    :-  beacon(Mac,Range)
		& beaconName(Mac,Location)
		& Range < 0.5.
		
move(sright)
	:-	irWall(Distance,Angle)
		& Distance > 11.

move(sleft)
	:-	irWall(Distance,Angle)
		& Distance < 9.

move(forward)
	:-	not (move(sright) | move(sleft)).

