
movement(waypoint).
+!waypoint(Location)
    :   at(Location)
    <-  drive(stop).

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
		& (Distance > 11.0).

move(sleft)
	:-	irWall(Distance,Angle)
		& (Distance < 9.0).

move(forward)
	:-	not (move(sright) | move(sleft)).

