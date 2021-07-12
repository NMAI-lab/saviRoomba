// Component behaviours (think in terms of state)

movement(waypoint).
+!waypoint(Location)
    :   atLocation(Location,_)
    <-  .broadcast(tell, waypoint(atLocation(Location)));
		drive(stop).

+!waypoint(Location)
	:	locationName(Location,[X,Y])
    <-  .broadcast(tell, waypoint(Location));
		!faceNext(Location);
		!drive(forward);
		-position(_,_);
		+position(X,Y);
		!waypoint(Location).

+!waypoint(Location)
	<-	.broadcast(tell, waypoint(Location,default)).
	
movement(faceNext).	
+!faceNext(Next)
	:	direction(CurrentDirection)
		& directionToNext(Next,NewDirection)
		& face(CurrentDirection,NewDirection,TurnAction)
	<-	.broadcast(tell, faceNext(Next));
		!turn(TurnAction);
		-direction(_);
		+direction(NewDirection).
		
+!faceNext(Next)
	<-	.broadcast(tell, faceNext(Next,default)).
		
directionToNext(Next,e)
	:-	position(X1,_)
		& locationName(Next,[X2,_])
		& X1 < X2.

directionToNext(Next,w)
	:-	position(X1,_)
		& locationName(Next,[X2,_])
		& X1 > X2.	
		
directionToNext(Next,n)
	:-	position(_,Y1)
		& locationName(Next,[_,Y2])
		& Y1 < Y2.

directionToNext(Next,s)
	:-	position(_,Y1)
		& locationName(Next,[_,Y2])
		& Y1 > Y2.	
		

face(OldDirection,NewDirection,around)
	:-	((OldDirection == s) & (NewDirection == n))
		| ((OldDirection == n) & (NewDirection == s))
		| ((OldDirection == e) & (NewDirection == w))
		| ((OldDirection == w) & (NewDirection == e)).
	
face(OldDirection,NewDirection,left)
	:-	((OldDirection == s) & (NewDirection == e))
		| ((OldDirection == e) & (NewDirection == n))
		| ((OldDirection == n) & (NewDirection == w))
		| ((OldDirection == w) & (NewDirection == s)).
		
face(OldDirection,NewDirection,right)
	:-	((OldDirection == s) & (NewDirection == w))
		| ((OldDirection == w) & (NewDirection == n))
		| ((OldDirection == n) & (NewDirection == e))
		| ((OldDirection == e) & (NewDirection == s)).
		
face(OldDirection,NewDirection,ok)
	:-	OldDirection == NewDirection.	

movement(turn).	
+!turn(left)
	:	turnRate(X,Y)
	<-	.broadcast(tell, turn(left));
		!move(X,Y,2).

+!turn(right)
	: 	turnRate(X,Y)
	<-	.broadcast(tell, turn(right));
		!move(0-X,0-Y,2).

+!turn(around)
	:	turnRate(X,Y)
	<-	.broadcast(tell, turn(around));
		!move(X,Y,4).

+!turn(_)
	<-	.print(turn(default)).

movement(drive).	
+!drive(forward)
	:	driveRate(X,Y)
	<-	.broadcast(tell, drive(forwad));
		!move(X,Y,5).
	
+!drive(backward)
	:	driveRate(X,Y)
	<-	.broadcast(tell, drive(backward));
		!move(0-X,0-Y,5).
	
+!drive(_)
	<-	.print(drive(default)).
	
movement(move).
+!move(X,Y,Count)
	: 	Count > 0
	<-	.broadcast(tell, move(X,Y,Count));
		drivexy(X,Y);
		!move(X,Y,Count-1).
	
+!move(_,_,_)
	<-	.print(move(default)).

	
/**
 * Turning and driving rates
 */
turnRate(0,4.1).
driveRate(0.5,0).



