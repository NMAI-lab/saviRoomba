// Component behaviours (think in terms of state)


movement(waypoint).
+!waypoint(Location)
    :   atLocation(Location,Range)
    <-  .broadcast(tell, waypoint(atLocation(Location,Range)));
		drive(stop).

+!waypoint(Location)
    :   nearLocation(Location,Range)	
    <-  !pointToLocation(Location);
		.broadcast(tell, waypoint(nearLocation(Location,Range)));
		drivexy(0.1,0);
		!waypoint(Location).

+!waypoint(Location)
    <-	!pointToLocation(Location);
		.broadcast(tell, waypoint(driving(Location)));
		drivexy(0.5,0);
		!waypoint(Location).
	
+!pointToLocation(Location)
	:	locationBearing(Location,Bearing)
	<-	.broadcast(tell, pointToLocation(Location,Range)).
	
	
/**
 * Calculate the course correction
 */
courseCorrection(TargetBearing, Correction)
	:-	compass(CurrentBearing)
		& declanation(Declanation)
		& (Correction = TargetBearing - (CurrentBearing + Declanation)).
		
/**
 * Rule used for calculating the steering setting based on course correction
 * target bearing.
 * steeringSetting(TargetBearing, SteeringSetting)
 */ 

steeringSetting(TargetBearing, 1)
	:-	courseCorrection(TargetBearing, Correction)
		& (Correction >= 20).
 
steeringSetting(TargetBearing, -1)
	:-	courseCorrection(TargetBearing, Correction)
		& (Correction <= -20).
		
steeringSetting(TargetBearing, Correction/180)
	:-	courseCorrection(TargetBearing, Correction)
		& (Correction < 20)
		& (Correction > -20).
		
		
locationBearing(Location,Bearing) 
	:- 	locationName(Location,[X,Y])
		& position(X,Y)
		& Bearing = math.atan(X/Y).
