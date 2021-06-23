
movement(waypoint).
+!waypoint(Location)
    :   at(Location)
    <-  drive(stop).

	
+!waypoint(Location)
    :   beacon(Location, Range, Direction)
        & (not at(Location))
    <-  drive(Direction).


at(Location)
    :-  beacon(Location,Range,_)
        & (Range < 5).
