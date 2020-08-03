/**
 * @author	Chidiebere Onyedinma
 * @author	Patrick Gavigan
 * @date	3 August 2020
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

// Arrived at the destination
atDestination :-
	(destination(DESTINATION) & postPoint(DESTINATION,_)) | direction(arrived).

// Destination is the previously seen post point
DestinationBehind :-
	(destinaton(DESTINATION) & postPoint(_,DESTINATION)) | direction(behind).

DestinationAhead :- 
	direction(forward).
	
DestinationRight :-
	direction(right).
	
DestinationLeft :-
	direction(left).

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
		not postPoint(SENDER,_) &
		batteryOK)
	<- 	!setDestination(SENDER);
		!goToLocation;
		!deliverMail.
		
// Case where I am at the sender location
// Assume that the fact that I have arrived at the sender location means that 
// I have the mail (this will need to be updated)
 +!deliverMail
 	: 	((not haveMail) &
		senderLocation(SENDER) &
		receiverLocation(RECEIVER) &
		postPoint(SENDER,_) & 
		batteryOK)
	<- 	+haveMail;
		!deliverMail.
 
// Case where I have the mail and need to deliver it to the receiver
 +!deliverMail
 	: 	(haveMail &
		receiverLocation(RECEIVER) &
		not postPoint(RECEIVER,_) &
		batteryOK)
	<- 	!setDestination(RERCEIVER);
		!goToLocation;
		!deliverMail.
		
// Case where I have the mail and am at the receiver location
 +!deliverMail
 	: 	(haveMail &
		receiverLocation(RECEIVER) &
		postPoint(RECEIVER,_) &
		batteryOK)
	<- 	-haveMail.

// Case where the battery is low
 +!deliverMail
 	: 	(batteryLow &
		dockStation(DOCK))	
	<-	-destination(_);
		+destination(DOCK);
		!dock;
		!deliverMail.
		
// Catchall (suspect that this should not be needed)
+!deliverMail
	<-	!deliverMail.

/** 
 * goToLocation
 * This is where the path planning stuff happens, deciding how to get to the
 * destination.
 */
+!goToLocation
	:	destinationAhead
	<-	!followPath.

+!goToLocation
	:	atDestination
	<-	drive(stop).
	
+!goToLocation
	:	destinationLeft	// TODO: Update to use unification for left, right, behind?
	<-	turn(left);	// TODO: This (or something similar) needs to be implementd
		!followPath.
		
+!goToLocation
	:	destinationRight	// TODO: Update to use unification for left, right, behind?
	<-	turn(right);		// TODO: This (or something similar) needs to be implementd
		!followPath.
	
+!goToLocation
	:	destinationBehind	// TODO: Update to use unification for left, right, behind?
	<-	turn(left);		// TODO: This (or something similar) needs to be implementd
		!followPath.

+!goToLocation.


/** 
 * followPath
 * Follow the line.
 */
 
 // Ideally, these plans could be combined using unification (see the last plan
 // in this set). This would need a modification of the scripts that interpret 
 // drive() action, or the script that generates the line() message (of both)
+!followPath
	:	line(center)
	<-	drive(forward);
		!followPath.
		
+!followPath
	:	line(lost)
	<-	drive(stop).

// Handle cases for left and right turns.
+!followPath
	:	line(DIRECTION)
	<-	drive(DIRECTION);
		!followPath.
		
/**
 * dock
 * Dock the robot at the charging station
 */
+!dock
 	:	dockStation(DOCK) & not postPoint(DOCK,_)
	<-	!goToLocation;
		!dock.

+!dock
	:	dockStation(DOCK) & postPoint(DOCK,_)
	<-	drive(stop);
		dock_bot.

		
/**
 * Set the destination of the robot
 */
+!setDestination(DESTINATION)
	<-	-destination(_);
		+destination(DESTINATION);
		setDestination(DESTINATION).
    