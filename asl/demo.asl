/**
 * @author	Chidiebere Onyedinma
 * @author	Patrick Gavigan
 * @date	2 July 2020
 */
 
/**
 * Main beliefs for the robot
 * Mission is hard coded for now (need to update this)
 */
senderLocation(post1).		// The location of the mail sender
receiverLocation(post4).	// The location of the mail receiver
dockStation(post5).			// The location of the docking station

/**
 * Navigation rules
 */
destAhead :-
	destination(N) &
	postPoint(C,P) &
	(N > C).

destBehind :-
	destination(N) &
	postPoint(C,P) &
	(N < C).
	
atDestination :-
	destination(N) &
	postPoint(C,P) &
	(N = C).

onTrack :-
	postPoint(_,P) &
	destination(N) &
	(((N > P) & destAhead) | ((N < P) & destBehind)) &
	line(center).
	
/**
 * High level goals
 */
!deliverMail.		// Highest level task: Deliver mail from sender to receiver
//!goToLocation.	// Go to a destination location (such as a post point)
//!followPath.		// Follow the path (line on the ground) 
//!dock.			// Dock the robot when it is time to recharge

/**
 * deliverMail
 * Go get the mail from the sender, deliver it to the receiver if I have it
 */
 
 // Case where I have a sender location and don't yet have the mail, not 
 // currently at the senderLocation.
 +!deliverMail
 	: 	((not haveMail) &
		senderLocation(SENDER) &
		receiverLocation(RECEIVER) &
		currentLocation(SENDER) & 
		batteryOK)
	<- 	+destination(SENDER);
		!goToLocation;
		!deliverMail.
		
// Case where I am at the sender location
// Assume that the fact that I have arrived at the sender location means that 
// I have the mail (this will need to be updated)
 +!deliverMail
 	: 	((not haveMail) &
		senderLocation(SENDER) &
		receiverLocation(RECEIVER) &
		currentLocation(SENDER) & 
		batteryOK)
	<- 	+haveMail;
		//-senderLocation(_);	// Should we remove the sender location here?
		!deliverMail.
 
// Case where I have the mail and need to deliver it to the receiver
 +!deliverMail
 	: 	(haveMail &
		receiverLocation(RECEIVER) &
		not currentLocation(RECEIVER) &
		batteryOK)
	<- 	!goToLocation;
		!deliverMail.
		
// Case where I have the mail and am at the receiver location
 +!deliverMail
 	: 	(haveMail &
		receiverLocation(RECEIVER) &
		currentLocation(RECEIVER) &
		batteryOK)
	<- 	-haveMail.
		// -receiverLocation(_).	// Should we remove the receiver location here?

// Case where the battery is low
 +!deliverMail
 	: 	(batteryLow &
		dockStation(DOCK))	
	<-	-destination(_);
		+destination(DOCK);
		!goToLocation.
		
// Catchall (suspect that this should not be needed)
+!deliverMail.

/** 
 * goToLocation
 * This is where the path planning stuff happens, deciding how to get to the
 * destination.
 */
 // TO DO: ADD THE NAVIGATION RULES HERE
 // THESE ARE FROM THE ORIGINAL CODE AND NEED TO BE UPDATED
+!goToLocation
	:	onTrack
	<-	!followPath.

+!goToLocation
	:	atDestination
	<-	drive(stop).

+!goToLocation
	:	batteryLow
	<-	!dock.

+!goToLocation
	:	batteryOK & docked
	<-	!undock.

+!goToLocation.


/** 
 * followPath
 * Follow the line.
 */
+!followPath
	:	line(center)
	<-	drive(forward);
		!followPath.
		
+!followPath
	:	line(right)
	<-	drive(right);
		!followPath.
		
+!followPath
	:	line(left)
	<-	drive(left);
		!followPath.
		
+!followPath
	:	line(across)
	<-	drive(stop);
		!followPath.

+!followPath
	:	line(lost)
	<-	drive(stop);
		!followPath.
		
 
/**
 * dock
 * Dock the robot at the charging station
 */
+!dock
 	:	not atDockPost & onTrack
	<-	!navigate;
		!dock.

+!dock
	:	atDockPost & moving
	<-	drive(stop);
    	!dock.

+!dock
	:	atDockPost & not moving
	<-	dock_bot.
	
+!dock. 

