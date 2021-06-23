



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
		& locationName(Destination,[DestLat,DestLon])
		& nearestLocation(Current,Range)
	<-	.broadcast(tell, navigate(gettingRoute(Destination), Range));
		.broadcast(tell, navigate(current(Current), CurrentRange));
		?a_star(Current,Destination,Solution,Cost);
		.broadcast(tell, navigate(route(Solution,Cost), Destination, Range));
		for (.member( op(drive,NextPosition), Solution)) {
			!waypoint(NextPosition);
		}
		!navigate(Destination).	
		
 // !navigate(Destination) - should be impossible
 +!navigate(Destination)
 	<-	.broadcast(tell, navigate(default, Destination)).

			
/**
 * A* Rules and Beliefs
 */
 
// A* Nav Rules
{ include("D:/Local Documents/ROS_Workspaces/saviRoomba/asl/a_star.asl") }

suc(Current,Next,Range,drive)
	:-	possible(Current,Next)
		& range(Current,Next,Range).
	
// heutistic definition: h(CurrentState,Goal,H)
h(Current,Goal,Range) 
	:-	range(Current,Goal,Range).
						 
range(A,B,Range)
	:-	locationName(A,[X1,Y1])
		& locationName(B,[X2,Y2])
		& Range = math.sqrt( ((X2-X1) * (X2-X1)) + ((Y2-Y1) * (Y2-Y1)) ).
