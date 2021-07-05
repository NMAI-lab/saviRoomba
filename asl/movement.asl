
movement(waypoint).
+!waypoint(Location)
    :   at(Location)
    <-  drive(stop).

	
// irWall(Distance,Angle)
// beacon(Mac,Range)
// position(X,Y)
// at(Location)
// range(A,B,Range)
	
!waypoint(c).

+!waypoint(Location)
    :   (not at(Location))
    <-  drive(forward).

at(Location)
    :-  beaconName(Mac,Location)
		& beacon(Mac,Range)
		& Range < 0.5.

