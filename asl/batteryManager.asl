batteryMin(0.50).
batteryMax(0.95).

/**
 * When the battery perception is below the minimum, we need to pickup the goal
 * of !chargeBattery, if we haven't already.
 */

health(battery).
 
// Check if I'm on a mission, if so, I'll need to launch the mission again
// once the battery is charged.
// Also need to confirm that I'm not already charging the battery. 
+battery(State)
	:	charging(false)
		& lowBattery(State)
		& mission(Goal,Parameters)
		& (not managingBattery)
	<-	.drop_all_intentions;
		.broadcast(tell, battery(chargingNeeded));
		!chargeBattery;
		.broadcast(tell, battery(chargingFinished));
		!mission(Goal,Parameters).

// No mission on the go, just need to charge the battery if I'm not already
// dealing with it.
+battery(State)
	:	charging(false)
		& lowBattery(State)
		& (not managingBattery)
	<-	.drop_all_intentions;
		.broadcast(tell, battery(chargingNeeded));
		!chargeBattery;
		.broadcast(tell, battery(chargingFinished)).
	
/**
 * !chargeBattery
 * The plan for getting the battery to battery(full) if needed.
 */
 
health(chargeBattery).
 
 // I'm not docked. Go to the docking station and dock.
 // This is rerursiuve as we need to wait for the battery to charge.
 +!chargeBattery
	:	lowBattery(_)
		& charging(false)
		& chargerLocation(ChargeStation)
	<-	.broadcast(tell, chargeBattery(chargingNeeded));
		+managingBattery;
		!navigate(ChargeStation);
		.broadcast(tell, chargeBattery(atDock));
		station(dock);
		.broadcast(tell, chargeBattery(docked));
		!chargeBattery.
		
// Battery is full, undock the robot
+!chargeBattery
	: 	fullBattery(_)
		& charging(true)
	<-	.broadcast(tell, chargeBattery(charged));
		station(undock);
		-managingBattery;
		.broadcast(tell, chargeBattery(unDocked)).
		
+!chargeBattery 
	<- 	.broadcast(tell, chargeBattery(waiting));
		!chargeBattery.

lowBattery(State)
	:-	battery(State)
		& batteryMin(Min)
		& State <= Min.

fullBattery(State)
	:-	battery(State)
		& batteryMax(State).
		
atStation
	:- 	position(X,Y)
		& locationName(ChargeStation,[X,Y])
		& chargerLocation(ChargeStation).
