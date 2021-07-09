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
	:	rotationSetting(Location,Rotation)
		& (math.abs(Rotation) > 0.1)
	<-	.broadcast(tell, pointToLocation(Location,Rotation));
		drivexy(0,Rotation);
		!pointToLocation(Location).
	
+!pointToLocation(Location)
	<-	.broadcast(tell, pointToLocation(Location,onTarget)).
	

/**
 * Rule used for calculating the steering setting based on course correction
 * and target name.
 */ 
rotationSetting(TargetName, 0.8)
	:-	courseCorrection(TargetName, Correction)
		& (Correction >= 10).
 
rotationSetting(TargetName, -0.8)
	:-	courseCorrection(TargetName, Correction)
		& (Correction <= -10).
		
rotationSetting(TargetName, 0)
	:-	courseCorrection(TargetName, Correction)
		& (Correction < 10)
		& (Correction > -10).
		
/**
 * Calculate the course correction
 */
courseCorrection(Location, Angle)
	:-	odomYaw(CurrentBearing)
		& locationBearing(Location,TargetBearing) 
		& (Angle = TargetBearing - CurrentBearing).	
		
locationBearing(Location,Bearing) 
	:- 	locationName(Location,[X2,Y2])
		& position(X1,Y1)
		& savi_ros_java.savi_ros_bdi.navigation.bearingXY(X1,Y1,X2,Y2,Bearing).
