// Component behaviours (think in terms of state)


// Turn behaviour:
// Direction you are travelling -> Direction vector: where you are - where you were
// Where you need to go: where you are going next = where you are
// -- use the differene in these vectors to generate a turn


// follow wall: if the wall is within 15 cm, try to follow it.
// otherwise, no wall, drive straight and hope for the best

// Need to recover from going the wrong way
// where I was (visited belief)
// Where I am going (goal)
// If where I end up is not where I expected to be, something went wrong... turn (see above)



movement(waypoint).
+!waypoint(Location)
    :   atLocation(Location,_)
    <-  drive(stop).

+!waypoint(Location)
    :   (not at(Location))		
    <-  !followWall
		!waypoint(Location).

+!waypoint(Location)
    <-  !waypoint(Location).
	
	
/**
 * Follow the wall if it is visible. Adjust course as needed to follow the wall.
 */
+!followWall
	:	wallRotation(Rotation)
		& Rotation > 10
	<-	drivexy(0.0,0.5);
		!followWall.
	
+!followWall
	:	wallRotation(Rotation)
		& Rotation < -10
	<-	drivexy(0.0,-0.5);
		!followWall.

+!followWall
	<-	drive(forward);
		!followWall.

// Wall range rules
wallRange(tooClose)
	:-	irWall(Distance,_)
		& (Distance < 9.0).
		
wallRange(tooFar)
	:-	irWall(Distance,_)
		& (Distance > 11.0)
		& (not wall(missing)).
		
wallRange(missing)
	:-	irWall(Distance,_)
		& (Distance > 20.0).
		
wallRange(correct)
	:-	(not wall(missing))
		& (not wall(tooFar))
		& (not wall(tooClose))
		& irWall(_,_).
	
// Wall angle rule - too close, turn away from the wall a bit
wallRotation(Rotation)
	:-	wallRange(tooClose)
		& irWall(_,Angle)
		& (Rotation = (100 - Angle)).
 
// Wall angle rule - too far, turn toward the wall a bit
wallRotation(Rotation)
	:-	wallRange(tooFar)
		& irWall(_,Angle)
		& (Rotation = (80 - Angle)).

// Scenario 3: Range from wall OK (between 9.0 and 11.0)
wallRotation(Rotation)
	:-	irWall(_,Angle)
		& wallRange(correct)
		& (Rotation = (90 - Angle)).	
	
// Old stuff below
		
move(sright)
	:-	irWall(Distance,Angle)
		& (Distance > 11.0).

move(sleft)
	:-	irWall(Distance,Angle)
		& (Distance < 9.0).

move(forward)
	:-	not (move(sright) | move(sleft)).

