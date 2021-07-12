+!mission(Goal,Parameters)
	:	Goal = navigate
		& Parameters = [Destination]
	<-	+mission(Goal,Parameters);
		!navigate(Destination);
		-mission(Goal,Parameters).

/**
 * !navigate(Destination)
 * Used for setting up the navigation path to get from the current location to 
 * the destination.
 * Beliefs: Relevant map definitions in map.asl
 * Actions: None
 * Goals Adopted: !driveToward(Location)
 */

navigation(navigate).

 // Case where we are already at the destination
+!navigate(Destination)
	:	atLocation(Destination, Range)
	<-	.broadcast(tell, navigate(arrived(Destination,Range))).

// We don't have a route plan, get one and set the waypoints.
+!navigate(Destination)
	:	(not atLocation(Destination,_))
		& atLocation(Current,Range)
	<-	.broadcast(tell, navigate(gettingRoute(Destination), Range));
		.broadcast(tell, navigate(current(Current), Range));
		?a_star(Current,Destination,Solution,Cost);
		.broadcast(tell, navigate(route(Solution,Cost), Destination, Range));
		for (.member( op(drive,NextPosition), Solution)) {
			!waypoint(NextPosition);
		}
		!navigate(Destination).	
		
 // !navigate(Destination) - should be impossible
 +!navigate(Destination)
 	<-	.broadcast(tell, navigate(default, Destination)).

// A* Nav Rules
{ include("/home/pi/create_ws/src/saviRoomba/asl/a_star.asl") }
